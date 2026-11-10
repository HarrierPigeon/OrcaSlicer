#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r::ShakeTune {

// How the two XY motors map onto toolhead motion. Shake&Tune measures each motor on its own by
// moving the toolhead along that motor's direction only: 0 and 90 degrees on a Cartesian machine
// (CoreXZ is measured the same way), 45 and 135 degrees on a CoreXY.
enum class MotorKinematics { Cartesian, CoreXY };

// A range of motor speeds, in mm/s, where a motor resonates. Shake&Tune expresses a motor's speed
// as the toolhead speed of a move along that motor's own direction, and so do these bands.
struct SpeedBand
{
    double lo{ 0. };
    double hi{ 0. };
    // Open interval: a speed exactly on an edge is outside the band.
    bool contains(double speed) const { return speed > lo && speed < hi; }
    bool operator==(const SpeedBand &rhs) const { return lo == rhs.lo && hi == rhs.hi; }
};
using SpeedBands = std::vector<SpeedBand>;

// Text form used by the printer settings: "45-52, 96.5-104". Bands are sorted and overlapping or
// touching ones merged. An empty string is no bands. Returns false and sets `error` on bad input.
bool        parse_speed_bands(const std::string &text, SpeedBands &bands, std::string *error = nullptr);
std::string format_speed_bands(const SpeedBands &bands);
// Sort by lower edge and merge bands that overlap or touch.
void        normalize_speed_bands(SpeedBands &bands);

// Speed of each motor, as a fraction of the toolhead speed, for a move along (dx, dy).
// Follows Shake&Tune's projection: Cartesian |cos|, |sin|; CoreXY |cos+sin|/sqrt2, |cos-sin|/sqrt2.
std::array<double, 2> motor_speed_factors(MotorKinematics kinematics, double dx, double dy);

// Whether neither motor runs inside one of its bands at toolhead speed `speed` along (dx, dy).
bool clears_resonance_bands(MotorKinematics kinematics, const std::array<SpeedBands, 2> &bands, double speed, double dx, double dy);

// The fastest toolhead speed, no faster than `speed`, at which neither motor runs inside one of its
// bands for a move along (dx, dy). A speed that already clears the bands is returned unchanged, and
// so is one that could only be cleared by dropping below `min_ratio * speed`.
double resonance_safe_speed(MotorKinematics kinematics, const std::array<SpeedBands, 2> &bands,
                            double speed, double dx, double dy, double min_ratio = 0.5);

// The slowest toolhead speed, no slower than `speed` and no faster than `max_speed`, at which
// neither motor runs inside one of its bands for a move along (dx, dy); nullopt if there is none.
std::optional<double> resonance_safe_speed_above(MotorKinematics kinematics, const std::array<SpeedBands, 2> &bands,
                                                 double speed, double dx, double dy, double max_speed);

} // namespace Slic3r::ShakeTune
