#include "MotorResonance.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

namespace Slic3r::ShakeTune {

namespace {

bool parse_number(const std::string &s, double &out)
{
    if (s.empty())
        return false;
    char *end = nullptr;
    out       = std::strtod(s.c_str(), &end);
    return end == s.c_str() + s.size() && std::isfinite(out);
}

std::string format_number(double v)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f", v);
    std::string s(buf);
    // "52.0" -> "52": whole speeds read better without a trailing zero.
    if (s.size() > 2 && s.compare(s.size() - 2, 2, ".0") == 0)
        s.resize(s.size() - 2);
    return s;
}

} // namespace

bool parse_speed_bands(const std::string &text, SpeedBands &bands, std::string *error)
{
    auto fail = [error](const std::string &msg) {
        if (error)
            *error = msg;
        return false;
    };
    SpeedBands               out;
    std::vector<std::string> items;
    boost::split(items, text, boost::is_any_of(",;"));
    for (std::string item : items) {
        boost::trim(item);
        if (item.empty())
            continue;
        // The separator is the first '-' after the first character, so a leading sign is not
        // taken for it; negative speeds are rejected below anyway.
        const size_t dash = item.find('-', 1);
        if (dash == std::string::npos)
            return fail("\"" + item + "\" is not a range such as 45-52");
        std::string lo_s = item.substr(0, dash), hi_s = item.substr(dash + 1);
        boost::trim(lo_s);
        boost::trim(hi_s);
        double lo, hi;
        if (!parse_number(lo_s, lo) || !parse_number(hi_s, hi))
            return fail("\"" + item + "\" is not a range such as 45-52");
        if (lo < 0. || hi <= lo)
            return fail("\"" + item + "\" must run from a lower to a higher speed");
        out.push_back({ lo, hi });
    }
    normalize_speed_bands(out);
    bands = std::move(out);
    return true;
}

void normalize_speed_bands(SpeedBands &bands)
{
    std::sort(bands.begin(), bands.end(), [](const SpeedBand &a, const SpeedBand &b) { return a.lo < b.lo; });
    SpeedBands merged;
    for (const SpeedBand &b : bands) {
        if (!merged.empty() && b.lo <= merged.back().hi)
            merged.back().hi = std::max(merged.back().hi, b.hi);
        else
            merged.push_back(b);
    }
    bands = std::move(merged);
}

std::string format_speed_bands(const SpeedBands &bands)
{
    std::string out;
    for (const SpeedBand &b : bands) {
        if (!out.empty())
            out += ", ";
        out += format_number(b.lo) + "-" + format_number(b.hi);
    }
    return out;
}

std::array<double, 2> motor_speed_factors(MotorKinematics kinematics, double dx, double dy)
{
    const double len = std::hypot(dx, dy);
    if (len <= 0.)
        return { 0., 0. };
    const double c = dx / len, s = dy / len;
    if (kinematics == MotorKinematics::CoreXY) {
        constexpr double inv_sqrt2 = 0.70710678118654752440;
        return { std::abs(c + s) * inv_sqrt2, std::abs(c - s) * inv_sqrt2 };
    }
    return { std::abs(c), std::abs(s) };
}

namespace {

// Whether neither motor runs inside one of its bands at toolhead speed v.
bool clears_bands(const std::array<SpeedBands, 2> &bands, const std::array<double, 2> &factor, double v)
{
    for (size_t m = 0; m < 2; ++m)
        for (const SpeedBand &b : bands[m])
            if (b.contains(v * factor[m]))
                return false;
    return true;
}

} // namespace

bool clears_resonance_bands(MotorKinematics kinematics, const std::array<SpeedBands, 2> &bands, double speed, double dx, double dy)
{
    return clears_bands(bands, motor_speed_factors(kinematics, dx, dy), speed);
}

double resonance_safe_speed(MotorKinematics kinematics, const std::array<SpeedBands, 2> &bands,
                            double speed, double dx, double dy, double min_ratio)
{
    if (!(speed > 0.))
        return speed;
    const std::array<double, 2> factor = motor_speed_factors(kinematics, dx, dy);
    auto clear = [&](double v) { return clears_bands(bands, factor, v); };
    if (clear(speed))
        return speed;
    // The speed only has to drop to the lower edge of whichever band a motor sits in, so the
    // candidates are those edges mapped back to toolhead speed. The fastest one that clears both
    // motors wins.
    std::vector<double> candidates;
    const double        floor = min_ratio * speed;
    for (size_t m = 0; m < 2; ++m) {
        if (factor[m] <= 1e-9)
            continue;
        for (const SpeedBand &b : bands[m]) {
            // Just under the edge, so rounding cannot put the motor back on the inside.
            const double v = b.lo / factor[m] - 1e-6;
            if (v < speed && v >= floor)
                candidates.push_back(v);
        }
    }
    std::sort(candidates.begin(), candidates.end(), std::greater<double>());
    for (double v : candidates)
        if (clear(v))
            return v;
    return speed;
}

std::optional<double> resonance_safe_speed_above(MotorKinematics kinematics, const std::array<SpeedBands, 2> &bands,
                                                 double speed, double dx, double dy, double max_speed)
{
    if (!(speed > 0.))
        return std::nullopt;
    const std::array<double, 2> factor = motor_speed_factors(kinematics, dx, dy);
    if (clears_bands(bands, factor, speed))
        return speed;
    // Mirror of resonance_safe_speed(): the candidates are the bands' upper edges.
    std::vector<double> candidates;
    for (size_t m = 0; m < 2; ++m) {
        if (factor[m] <= 1e-9)
            continue;
        for (const SpeedBand &b : bands[m]) {
            const double v = b.hi / factor[m] + 1e-6;
            if (v > speed && v <= max_speed)
                candidates.push_back(v);
        }
    }
    std::sort(candidates.begin(), candidates.end());
    for (double v : candidates)
        if (clears_bands(bands, factor, v))
            return v;
    return std::nullopt;
}

} // namespace Slic3r::ShakeTune
