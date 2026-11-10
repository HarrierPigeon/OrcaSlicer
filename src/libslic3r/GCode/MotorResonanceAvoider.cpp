#include "MotorResonanceAvoider.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <optional>
#include <string_view>
#include <system_error>
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

// The value of " KEY=value" in a marker line.
std::optional<double> marker_value(std::string_view line, std::string_view key)
{
    for (size_t pos = line.find(key); pos != std::string_view::npos; pos = line.find(key, pos + 1)) {
        const size_t value = pos + key.size() + 1;
        if (pos > 0 && line[pos - 1] == ' ' && value <= line.size() && line[value - 1] == '=') {
            double     v   = 0.;
            const auto res = fast_float::from_chars(line.data() + value, line.data() + line.size(), v);
            return res.ec == std::errc() ? std::optional<double>(v) : std::nullopt;
        }
    }
    return std::nullopt;
}

MotorResonanceAvoider::PathLimits parse_limits(std::string_view line)
{
    MotorResonanceAvoider::PathLimits l;
    l.mode             = marker_value(line, "MODE").value_or(0.) == 1. ? MotorResonanceAvoider::Mode::Nearest :
                                                                         MotorResonanceAvoider::Mode::SlowDown;
    l.role             = int(marker_value(line, "ROLE").value_or(0.));
    l.speed            = marker_value(line, "F").value_or(0.) / 60.;
    l.max_speed        = marker_value(line, "FMAX").value_or(0.) / 60.;
    l.acceleration     = (unsigned int) std::max(0., marker_value(line, "A").value_or(0.));
    l.max_acceleration = (unsigned int) std::max(0., marker_value(line, "AMAX").value_or(0.));
    return l;
}

// Moves still within this of the speed their path was generated at were not slowed down (mm/s).
constexpr double UNSLOWED_TOLERANCE = 0.5;
// A raised acceleration is rounded up to a multiple of this (mm/s²).
constexpr double ACCELERATION_STEP = 100.;

} // namespace

std::string MotorResonanceAvoider::start_marker(const PathLimits &limits)
{
    char buf[192];
    std::snprintf(buf, sizeof(buf), "%s MODE=%d ROLE=%d F=%.1f FMAX=%.1f A=%u AMAX=%u", START_MARKER,
                  limits.mode == Mode::Nearest ? 1 : 0, limits.role, limits.speed * 60., limits.max_speed * 60.,
                  limits.acceleration, limits.max_acceleration);
    return buf;
}

MotorResonanceAvoider::MotorResonanceAvoider(Config config) : m_config(std::move(config)) {}

std::vector<bool> MotorResonanceAvoider::holds_speed(const std::vector<Move> &moves, const std::vector<double> &speeds,
                                                     double acceleration, double square_corner_velocity)
{
    // Entry speed of each move, with the path starting and ending at rest. Corners are taken at
    // Klipper's junction speed for its square corner velocity, which only depends on the angle.
    const size_t        n = moves.size();
    std::vector<double> v(n + 1, 0.);
    for (size_t i = 1; i < n; ++i) {
        const Move  &a         = moves[i - 1];
        const Move  &b         = moves[i];
        const double cos_theta = std::clamp(-(a.dx * b.dx + a.dy * b.dy) / (a.length * b.length), -1., 1.);
        const double sin_half  = std::sqrt(0.5 * (1. - cos_theta));
        double       junction  = std::numeric_limits<double>::max();
        if (sin_half < 1. - 1e-9)
            junction = square_corner_velocity * std::sqrt((std::sqrt(2.) - 1.) * sin_half / (1. - sin_half));
        v[i] = std::min({ junction, speeds[i - 1], speeds[i] });
    }
    for (size_t i = n; i-- > 0;)
        v[i] = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2. * acceleration * moves[i].length));
    for (size_t i = 0; i < n; ++i)
        v[i + 1] = std::min(v[i + 1], std::sqrt(v[i] * v[i] + 2. * acceleration * moves[i].length));

    std::vector<bool> holds(n);
    for (size_t i = 0; i < n; ++i) {
        const double s     = speeds[i];
        const double ramps = (2. * s * s - v[i] * v[i] - v[i + 1] * v[i + 1]) / (2. * acceleration);
        holds[i]           = moves[i].length - ramps >= MIN_CRUISE_SHARE * moves[i].length;
    }
    return holds;
}

std::vector<MotorResonanceAvoider::Piece> MotorResonanceAvoider::analyse(const std::vector<std::string_view> &lines, double &layer_time)
{
    std::vector<Piece>    pieces;
    std::optional<size_t> open;
    layer_time = 0.;
    for (size_t li = 0; li < lines.size(); ++li) {
        const std::string_view line = lines[li];
        if (starts_with(line, START_MARKER)) {
            Piece p;
            p.start_line = li;
            p.end_line   = lines.size();
            p.limits     = parse_limits(line);
            pieces.push_back(std::move(p));
            open = pieces.size() - 1;
            continue;
        }
        if (starts_with(line, END_MARKER)) {
            if (open) {
                pieces[*open].end_line = li;
                pieces[*open].closed   = true;
            }
            open.reset();
            continue;
        }
        const std::string_view code = line.substr(0, line.find(';'));
        if (starts_with(code, "G90") || starts_with(code, "G91"))
            m_relative_xy = code[2] == '1';
        else if (starts_with(code, "M82"))
            m_relative_e = false;
        else if (starts_with(code, "M83"))
            m_relative_e = true;

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
            continue;
        }
        if (move.g == 92) {
            if (move.e)
                m_e = move.e->value;
            if (move.x)
                m_x = move.x->value;
            if (move.y)
                m_y = move.y->value;
        } else if (move.g == 28) {
            // Homing leaves the toolhead somewhere we cannot know.
            m_have_xy = false;
        }
        if (move.g > 3)
            continue;
        if (move.f)
            m_f = move.f->value;

        const double old_x = m_x, old_y = m_y;
        const bool   had_xy = m_have_xy;
        if (move.x)
            m_x = m_relative_xy ? m_x + move.x->value : move.x->value;
        if (move.y)
            m_y = m_relative_xy ? m_y + move.y->value : move.y->value;
        if (move.x && move.y && !m_relative_xy)
            m_have_xy = true;
        bool extruding = false;
        if (move.e) {
            extruding = m_relative_e ? move.e->value > 0. : move.e->value > m_e;
            m_e       = m_relative_e ? m_e + move.e->value : move.e->value;
        }

        const double dx = m_x - old_x, dy = m_y - old_y, length = std::hypot(dx, dy);
        if (length > 0. && m_f > 0.)
            layer_time += length / (m_f / 60.);
        if (open && move.g == 1 && extruding && had_xy && length > 0. && m_f > 0.)
            pieces[*open].moves.push_back(Move{ li, dx, dy, length, m_f / 60., m_f / 60. });
    }
    return pieces;
}

void MotorResonanceAvoider::decide(std::vector<Piece> &pieces, double layer_time) const
{
    // Below the band unless decided otherwise.
    for (Piece &p : pieces)
        for (Move &m : p.moves)
            m.target = ShakeTune::resonance_safe_speed(m_config.kinematics, m_config.bands, m.speed, m.dx, m.dy);
    std::map<int, std::vector<Piece *>> roles;
    for (Piece &p : pieces)
        if (p.limits.mode == Mode::Nearest && !p.moves.empty())
            roles[p.limits.role].push_back(&p);
    for (auto &[role, group] : roles)
        decide_nearest(group, layer_time);
}

void MotorResonanceAvoider::decide_nearest(std::vector<Piece *> &pieces, double &layer_time) const
{
    const double scv = m_config.square_corner_velocity > 0. ? m_config.square_corner_velocity : 5.;

    // Every move whose speed runs a motor inside a band, with the quiet speed above it when it may
    // go there: still at the speed its path was generated at, within the volumetric limit, and with
    // a known acceleration to plan with.
    struct Affected
    {
        Piece                *piece;
        size_t                index;
        bool                  unslowed;
        std::optional<double> up;
    };
    std::vector<Affected> affected;
    for (Piece *p : pieces)
        for (size_t i = 0; i < p->moves.size(); ++i) {
            const Move &m = p->moves[i];
            if (ShakeTune::clears_resonance_bands(m_config.kinematics, m_config.bands, m.speed, m.dx, m.dy))
                continue;
            Affected a{ p, i, p->limits.speed > 0. && m.speed >= p->limits.speed - UNSLOWED_TOLERANCE, std::nullopt };
            if (a.unslowed && p->limits.acceleration > 0)
                a.up = ShakeTune::resonance_safe_speed_above(m_config.kinematics, m_config.bands, m.speed, m.dx, m.dy,
                                                             p->limits.max_speed > 0. ? p->limits.max_speed :
                                                                                        std::numeric_limits<double>::max());
            affected.push_back(a);
        }

    // The moves that decide: unslowed ones holding their speed now. A slowed move goes below its
    // band whatever is decided.
    std::map<const Piece *, std::vector<bool>> holding_now;
    for (Piece *p : pieces) {
        std::vector<double> speeds;
        for (const Move &m : p->moves)
            speeds.push_back(m.speed);
        holding_now[p] = holds_speed(p->moves, speeds, std::max(1., double(p->limits.acceleration)), scv);
    }
    double up_cost = 0., down_cost = 0.;
    bool   any = false;
    for (const Affected &a : affected) {
        if (!a.unslowed || !holding_now[a.piece][a.index])
            continue;
        const Move &m = a.piece->moves[a.index];
        if (!a.up)
            return; // one of them cannot go above its band: all go below
        any = true;
        up_cost += m.length * (*a.up - m.speed);
        down_cost += m.length * (m.speed - m.target);
    }
    if (!any || up_cost >= down_cost)
        return;

    // Speeds with every unslowed affected move above its band, then the lowest acceleration at
    // which every deciding move holds its new speed.
    std::map<const Piece *, std::vector<double>> speeds_up;
    for (Piece *p : pieces) {
        std::vector<double> &s = speeds_up[p];
        for (const Move &m : p->moves)
            s.push_back(m.target);
    }
    double time_change = 0.;
    for (const Affected &a : affected)
        if (a.up) {
            const Move &m                    = a.piece->moves[a.index];
            speeds_up[a.piece][a.index]      = *a.up;
            time_change += m.length / *a.up - m.length / m.speed;
        }
    if (m_config.min_layer_time > 0. && layer_time + time_change < m_config.min_layer_time)
        return;

    double lowest = 0., highest = std::numeric_limits<double>::max();
    for (Piece *p : pieces) {
        lowest  = std::max(lowest, double(p->limits.acceleration));
        highest = std::min(highest, m_config.acceleration_command && p->closed && p->limits.max_acceleration > 0 ?
                                        double(p->limits.max_acceleration) : double(p->limits.acceleration));
    }
    highest       = std::max(highest, lowest);
    auto feasible = [&](double acceleration) {
        for (const Affected &a : affected) {
            if (!a.unslowed || !holding_now[a.piece][a.index])
                continue;
            const std::vector<bool> holds = holds_speed(a.piece->moves, speeds_up[a.piece],
                                                        std::max(acceleration, double(a.piece->limits.acceleration)), scv);
            if (!holds[a.index])
                return false;
        }
        return true;
    };
    double acceleration = lowest;
    if (!feasible(lowest)) {
        if (!feasible(highest))
            return;
        double lo = lowest, hi = highest;
        for (int i = 0; i < 24 && hi - lo > 1.; ++i) {
            const double mid = 0.5 * (lo + hi);
            (feasible(mid) ? hi : lo) = mid;
        }
        acceleration = std::min(highest, std::ceil(hi / ACCELERATION_STEP) * ACCELERATION_STEP);
    }

    for (const Affected &a : affected)
        if (a.up)
            a.piece->moves[a.index].target = *a.up;
    for (Piece *p : pieces)
        if (acceleration > p->limits.acceleration &&
            std::any_of(p->moves.begin(), p->moves.end(), [](const Move &m) { return m.target > m.speed; }))
            p->raised_acceleration = (unsigned int) acceleration;
    layer_time += time_change;
}

void MotorResonanceAvoider::emit_line(std::string_view line, const double *target, bool round_up, std::string &out)
{
    const std::string_view code = line.substr(0, line.find(';'));
    ParsedMove             move;
    if (!parse_move(code, move) || move.g < 0 || move.g > 3) {
        out.append(line);
        return;
    }
    if (move.f) {
        m_intended_f      = move.f->value;
        m_intended_f_text = std::string(line.substr(move.f->value_begin, move.f->value_end - move.f->value_begin));
    }
    // The feedrate this move should run at: whole mm/min, rounded away from the band it leaves.
    double      desired      = m_intended_f;
    std::string desired_text = m_intended_f_text;
    if (target) {
        const long f = round_up ? long(std::ceil(*target * 60.)) : long(std::floor(*target * 60.));
        desired      = double(f);
        desired_text = std::to_string(f);
    }
    if (move.f) {
        if (desired != move.f->value) {
            out.append(line.substr(0, move.f->value_begin));
            out += desired_text;
            out.append(line.substr(move.f->value_end));
        } else
            out.append(line);
    } else if (desired != m_emitted_f && !desired_text.empty()) {
        // Insert the feedrate after the last word, before any comment.
        size_t insert_at = code.size();
        while (insert_at > 0 && (code[insert_at - 1] == ' ' || code[insert_at - 1] == '\t'))
            --insert_at;
        out.append(line.substr(0, insert_at));
        out += " F";
        out += desired_text;
        out.append(line.substr(insert_at));
    } else
        out.append(line);
    m_emitted_f = desired;
}

std::string MotorResonanceAvoider::process_layer(std::string &&gcode)
{
    if (gcode.empty())
        return std::move(gcode);
    std::vector<std::string_view> lines;
    for (size_t pos = 0; pos < gcode.size();) {
        const size_t eol = gcode.find('\n', pos);
        if (eol == std::string::npos) {
            lines.emplace_back(gcode.data() + pos, gcode.size() - pos);
            break;
        }
        lines.emplace_back(gcode.data() + pos, eol - pos);
        pos = eol + 1;
    }
    const bool ends_with_newline = gcode.back() == '\n';

    double             layer_time = 0.;
    std::vector<Piece> pieces     = analyse(lines, layer_time);
    decide(pieces, layer_time);

    std::vector<const Piece *> start_of(lines.size(), nullptr), end_of(lines.size(), nullptr);
    std::vector<const Move *>  move_at(lines.size(), nullptr);
    for (const Piece &p : pieces) {
        start_of[p.start_line] = &p;
        if (p.end_line < lines.size())
            end_of[p.end_line] = &p;
        for (const Move &m : p.moves)
            move_at[m.line] = &m;
    }

    std::string out;
    out.reserve(gcode.size() + gcode.size() / 64);
    for (size_t li = 0; li < lines.size(); ++li) {
        // The markers are internal: their lines are dropped, leaving only a raised acceleration
        // and its restoration.
        if (const Piece *p = start_of[li]) {
            if (p->raised_acceleration > 0)
                out += m_config.acceleration_command(p->raised_acceleration);
            continue;
        }
        if (const Piece *p = end_of[li]) {
            if (p->raised_acceleration > 0)
                out += m_config.acceleration_command(p->limits.acceleration);
            continue;
        }
        if (starts_with(lines[li], END_MARKER))
            continue;
        const Move *m       = move_at[li];
        const bool  changed = m != nullptr && m->target != m->speed;
        emit_line(lines[li], changed ? &m->target : nullptr, changed && m->target > m->speed, out);
        if (li + 1 < lines.size() || ends_with_newline)
            out += '\n';
    }
    return out;
}

} // namespace Slic3r
