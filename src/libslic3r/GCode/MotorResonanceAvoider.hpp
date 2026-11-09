#pragma once

#include <array>
#include <string>

#include "../ShakeTune/MotorResonance.hpp"

namespace Slic3r {

// Keeps each XY motor out of the speed bands where it resonates (the cause of vertical fine
// artifacts), using bands measured with Shake&Tune's vibrations profile.
//
// It runs on finished layer G-code, after the CoolingBuffer: slowing a layer down for cooling
// rescales feedrates, which could put a motor back into a band. GCode::_extrude brackets the moves
// of every path that is in scope with START/END markers; between them, each G1 extrusion move is
// slowed to the fastest speed at which neither motor sits inside one of its bands, given the move's
// direction. The markers are removed, and the original feedrate is restored on the next move that
// relies on it.
class MotorResonanceAvoider
{
public:
    static constexpr const char *START_MARKER = ";_MOTOR_RESONANCE_START";
    static constexpr const char *END_MARKER   = ";_MOTOR_RESONANCE_END";

    MotorResonanceAvoider(ShakeTune::MotorKinematics kinematics, std::array<ShakeTune::SpeedBands, 2> bands);

    std::string process_layer(std::string &&gcode);

private:
    // Appends the (possibly rewritten) line to `out`; false when the line is dropped.
    bool process_line(const char *begin, const char *end, std::string &out);

    ShakeTune::MotorKinematics           m_kinematics;
    std::array<ShakeTune::SpeedBands, 2> m_bands;

    // Inside a START/END bracket.
    bool        m_active{ false };
    bool        m_relative_xy{ false };
    bool        m_relative_e{ true };
    bool        m_have_xy{ false };
    double      m_x{ 0. };
    double      m_y{ 0. };
    double      m_e{ 0. };
    // Feedrate the G-code asked for, and the text it was written as.
    double      m_intended_f{ 0. };
    std::string m_intended_f_text;
    // Feedrate the printer is actually at after the lines emitted so far.
    double      m_emitted_f{ 0. };
};

} // namespace Slic3r
