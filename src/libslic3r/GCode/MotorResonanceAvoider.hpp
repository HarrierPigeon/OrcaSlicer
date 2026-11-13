#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "../ShakeTune/MotorResonance.hpp"

namespace Slic3r {

// Keeps each XY motor out of the speed bands where it resonates (the cause of vertical fine
// artifacts), using bands measured with Shake&Tune's vibrations profile.
//
// It runs on finished layer G-code, after the CoolingBuffer: slowing a layer down for cooling
// rescales feedrates, which could put a motor back into a band. GCode::_extrude_piece brackets the
// moves of every path that is in scope with START/END markers, the START marker carrying what the
// path allows (PathLimits). Between them, each G1 extrusion move whose speed runs a motor inside a
// band, given the move's direction, is changed:
// - SlowDown: to the fastest speed below the band.
// - Nearest: once per layer for each role, either every such move goes below its band or every
//   one goes above it, whichever is nearer on average. Above is chosen only when every move of that
//   role holding its speed now can also reach and hold the faster one (a trapezoidal planner with
//   Klipper's junction speeds), with the path's acceleration raised up to the machine limit if that
//   is what it takes, and only for moves still at the speed they were generated at: a move slowed
//   for cooling or an overhang, or one that would break the volumetric limit, goes below. The
//   layer also may not become quicker than the minimum layer time.
// The markers are removed, a raised acceleration is set at the start of the path and restored at
// its end, and the original feedrate is restored on the next move that relies on it.
class MotorResonanceAvoider
{
public:
    static constexpr const char *START_MARKER = ";_MOTOR_RESONANCE_START";
    static constexpr const char *END_MARKER   = ";_MOTOR_RESONANCE_END";

    // How a speed inside a band is changed.
    enum class Mode { SlowDown, Nearest };

    // What a bracketed path allows, written after START_MARKER.
    struct PathLimits
    {
        Mode         mode{ Mode::SlowDown };
        // Paths of one role on one layer are decided together.
        int          role{ 0 };
        // Speed the path was generated at, before any slowdown for cooling or overhangs (mm/s).
        double       speed{ 0. };
        // Fastest the path may run, from the filament's volumetric limit (mm/s); 0 for no limit.
        double       max_speed{ 0. };
        // Acceleration in effect for the path, and the highest it may be raised to (mm/s²); 0 when
        // unknown or not to be raised.
        unsigned int acceleration{ 0 };
        unsigned int max_acceleration{ 0 };
    };
    static std::string start_marker(const PathLimits &limits);

    struct Config
    {
        ShakeTune::MotorKinematics               kinematics{ ShakeTune::MotorKinematics::Cartesian };
        std::array<ShakeTune::SpeedBands, 2>     bands;
        // Klipper's square corner velocity, for how fast the toolhead gets through corners (mm/s).
        double                                   square_corner_velocity{ 5. };
        // A layer may not become quicker than this (s); 0 for no minimum layer time.
        double                                   min_layer_time{ 0. };
        // G-code setting the print acceleration; without it the acceleration is never raised.
        std::function<std::string(unsigned int)> acceleration_command;
    };

    // Share of a move that has to run at its speed for the move to hold that speed.
    static constexpr double MIN_CRUISE_SHARE = 0.5;

    explicit MotorResonanceAvoider(Config config);

    std::string process_layer(std::string &&gcode);

private:
    // A G1 extrusion move inside a bracket.
    struct Move
    {
        size_t line{ 0 };
        double dx{ 0. }, dy{ 0. }, length{ 0. };
        // Speed the G-code asks for, and the one it will run at (mm/s).
        double speed{ 0. }, target{ 0. };
    };
    struct Piece
    {
        size_t            start_line{ 0 }, end_line{ 0 };
        // Its END marker is in the layer, where a raised acceleration can be restored.
        bool              closed{ false };
        PathLimits        limits;
        std::vector<Move> moves;
        // Set when the acceleration is raised for this piece.
        unsigned int      raised_acceleration{ 0 };
    };

    // Whether each move holds its speed in `speeds` for at least MIN_CRUISE_SHARE of its length,
    // with the path starting and ending at rest.
    static std::vector<bool> holds_speed(const std::vector<Move> &moves, const std::vector<double> &speeds, double acceleration,
                                         double square_corner_velocity);
    // Finds the bracketed pieces and their moves, and estimates how long the layer takes.
    std::vector<Piece> analyse(const std::vector<std::string_view> &lines, double &layer_time);
    // Sets each move's target speed and each piece's acceleration.
    void decide(std::vector<Piece> &pieces, double layer_time) const;
    void decide_nearest(std::vector<Piece *> &pieces, double &layer_time) const;
    // Emits a line, rewriting its feedrate to `target` (mm/s, rounded towards `round_up`) if set.
    void emit_line(std::string_view line, const double *target, bool round_up, std::string &out);

    Config m_config;

    // Where the analysis is, carried from one layer to the next.
    bool   m_relative_xy{ false };
    bool   m_relative_e{ true };
    bool   m_have_xy{ false };
    double m_x{ 0. };
    double m_y{ 0. };
    double m_e{ 0. };
    double m_f{ 0. };

    // Feedrate the G-code asked for, and the text it was written as.
    double      m_intended_f{ 0. };
    std::string m_intended_f_text;
    // Feedrate the printer is actually at after the lines emitted so far.
    double      m_emitted_f{ 0. };
};

} // namespace Slic3r
