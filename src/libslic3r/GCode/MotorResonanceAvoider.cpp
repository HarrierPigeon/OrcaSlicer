#include "MotorResonanceAvoider.hpp"

#include <cctype>
#include <cmath>
#include <cstring>
#include <optional>
#include <string_view>
#include <utility>

#include <fast_float/fast_float.h>

namespace Slic3r {

namespace {

struct Word
{
    double      value{ 0. };
    // Position of the value text within the line.
    size_t      value_begin{ 0 };
    size_t      value_end{ 0 };
};

struct ParsedMove
{
    int                 g{ -1 };
    std::optional<Word> x, y, e, f;
};

bool starts_with(std::string_view s, std::string_view prefix) { return s.substr(0, prefix.size()) == prefix; }

// Parses "G<n> <axis><value> ..." up to a comment. Returns false for anything else.
bool parse_move(std::string_view code, ParsedMove &move)
{
    size_t i = 0;
    while (i < code.size() && (code[i] == ' ' || code[i] == '\t'))
        ++i;
    if (i >= code.size() || (code[i] != 'G' && code[i] != 'g'))
        return false;
    ++i;
    int g = 0, digits = 0;
    while (i < code.size() && code[i] >= '0' && code[i] <= '9') {
        g = g * 10 + (code[i++] - '0');
        ++digits;
    }
    if (digits == 0 || (i < code.size() && code[i] != ' ' && code[i] != '\t'))
        return false;
    move.g = g;
    while (i < code.size()) {
        if (code[i] == ' ' || code[i] == '\t') {
            ++i;
            continue;
        }
        const char axis = char(code[i] & ~0x20); // upper case
        ++i;
        double      v   = 0.;
        const auto  res = fast_float::from_chars(code.data() + i, code.data() + code.size(), v);
        const size_t end = size_t(res.ptr - code.data());
        if (res.ec != std::errc()) {
            // Not a number: skip the word.
            while (i < code.size() && code[i] != ' ' && code[i] != '\t')
                ++i;
            continue;
        }
        Word w{ v, i, end };
        switch (axis) {
        case 'X': move.x = w; break;
        case 'Y': move.y = w; break;
        case 'E': move.e = w; break;
        case 'F': move.f = w; break;
        default: break;
        }
        i = end;
    }
    return true;
}

} // namespace

MotorResonanceAvoider::MotorResonanceAvoider(ShakeTune::MotorKinematics kinematics, std::array<ShakeTune::SpeedBands, 2> bands)
    : m_kinematics(kinematics), m_bands(std::move(bands))
{}

std::string MotorResonanceAvoider::process_layer(std::string &&gcode)
{
    if (gcode.empty())
        return std::move(gcode);
    std::string out;
    out.reserve(gcode.size() + gcode.size() / 64);
    const char *p   = gcode.data();
    const char *end = p + gcode.size();
    while (p < end) {
        const char *eol = static_cast<const char *>(std::memchr(p, '\n', size_t(end - p)));
        const char *next = eol ? eol + 1 : end;
        if (process_line(p, eol ? eol : end, out) && eol)
            out += '\n';
        p = next;
    }
    return out;
}

bool MotorResonanceAvoider::process_line(const char *begin, const char *end, std::string &out)
{
    const std::string_view line(begin, size_t(end - begin));
    // The markers are internal: drop their lines.
    if (starts_with(line, START_MARKER)) {
        m_active = true;
        return false;
    }
    if (starts_with(line, END_MARKER)) {
        m_active = false;
        return false;
    }
    const size_t      semicolon = line.find(';');
    const std::string_view code = line.substr(0, semicolon);

    if (starts_with(code, "G90") || starts_with(code, "G91")) {
        m_relative_xy = code[2] == '1';
    } else if (starts_with(code, "M82")) {
        m_relative_e = false;
    } else if (starts_with(code, "M83")) {
        m_relative_e = true;
    }

    ParsedMove move;
    if (!parse_move(code, move) || move.g < 0) {
        // A standard code is a letter and a number (M204, T0). Any other word is a firmware macro
        // (a timelapse frame, a custom command) that may move the toolhead: the next move's
        // direction is unknown until an absolute XY move pins the position down again. Klipper's
        // SET_* commands and EXCLUDE_OBJECT markers only change settings, and Orca writes them
        // between a travel and the extrusion that follows it.
        const size_t first = code.find_first_not_of(" \t\r");
        if (first != std::string_view::npos && std::isalpha((unsigned char) code[first]) &&
            !(first + 1 < code.size() && std::isdigit((unsigned char) code[first + 1]))) {
            const std::string_view word = code.substr(first);
            if (!starts_with(word, "SET_") && !starts_with(word, "EXCLUDE_OBJECT"))
                m_have_xy = false;
        }
        out.append(begin, end);
        return true;
    }
    if (move.g == 92) {
        if (move.e)
            m_e = move.e->value;
        if (move.x) m_x = move.x->value;
        if (move.y) m_y = move.y->value;
    } else if (move.g == 28) {
        // Homing leaves the toolhead somewhere we cannot know.
        m_have_xy = false;
    }
    if (move.g > 3) {
        out.append(begin, end);
        return true;
    }

    if (move.f) {
        m_intended_f      = move.f->value;
        m_intended_f_text = std::string(line.substr(move.f->value_begin, move.f->value_end - move.f->value_begin));
    }

    const double old_x = m_x, old_y = m_y;
    const bool   had_xy = m_have_xy;
    if (move.x) m_x = m_relative_xy ? m_x + move.x->value : move.x->value;
    if (move.y) m_y = m_relative_xy ? m_y + move.y->value : move.y->value;
    if (move.x && move.y && !m_relative_xy)
        m_have_xy = true;

    bool extruding = false;
    if (move.e) {
        extruding = m_relative_e ? move.e->value > 0. : move.e->value > m_e;
        m_e       = m_relative_e ? m_e + move.e->value : move.e->value;
    }

    // The feedrate this move should run at.
    double      desired      = m_intended_f;
    std::string desired_text = m_intended_f_text;
    if (m_active && move.g == 1 && extruding && (move.x || move.y) && had_xy && m_intended_f > 0.) {
        const double dx = m_x - old_x, dy = m_y - old_y;
        if (dx != 0. || dy != 0.) {
            const double v    = m_intended_f / 60.;
            const double safe = ShakeTune::resonance_safe_speed(m_kinematics, m_bands, v, dx, dy);
            if (safe < v) {
                // Whole mm/min, rounded down so the motor stays at or below the band's edge.
                const long f = long(std::floor(safe * 60.));
                desired      = double(f);
                desired_text = std::to_string(f);
            }
        }
    }

    if (move.f) {
        if (desired != move.f->value) {
            out.append(begin, begin + move.f->value_begin);
            out += desired_text;
            out.append(begin + move.f->value_end, end);
        } else
            out.append(begin, end);
    } else if (desired != m_emitted_f && !desired_text.empty()) {
        // Insert the feedrate after the last word, before any comment.
        size_t insert_at = code.size();
        while (insert_at > 0 && (code[insert_at - 1] == ' ' || code[insert_at - 1] == '\t'))
            --insert_at;
        out.append(begin, begin + insert_at);
        out += " F";
        out += desired_text;
        out.append(begin + insert_at, end);
    } else
        out.append(begin, end);
    m_emitted_f = desired;
    return true;
}

} // namespace Slic3r
