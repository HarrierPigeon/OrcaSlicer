#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <string>

#include "libslic3r/GCode/MotorResonanceAvoider.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/ShakeTune/MotorResonance.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

#include "test_helpers.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

struct WallSpeeds
{
    // Wall moves after the first layer, by role and by the axis they run along.
    int  outer_along_x{ 0 };
    int  outer_along_y{ 0 };
    int  inner_along_x{ 0 };
    // Moves whose X stepper speed (Cartesian: the X component of the speed) is inside the band.
    int  outer_x_in_band{ 0 };
    int  inner_x_in_band{ 0 };
    // Slowest Y-axis outer wall move, in mm/s.
    double outer_along_y_min_speed{ 1e9 };
};

WallSpeeds wall_speeds(const std::string &gcode, double band_lo, double band_hi)
{
    WallSpeeds  out;
    std::string role;
    int         layer = 0;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &r, const GCodeReader::GCodeLine &line) {
        const std::string_view raw = line.raw();
        if (raw.rfind(";TYPE:", 0) == 0)
            role = std::string(raw.substr(6));
        else if (raw.rfind(";LAYER_CHANGE", 0) == 0)
            ++layer;
        if (layer < 2 || !line.extruding(r) || (role != "Outer wall" && role != "Inner wall"))
            return;
        const double dx = std::abs(line.dist_X(r)), dy = std::abs(line.dist_Y(r));
        if (dx + dy < 1.)
            return;
        const double speed   = line.new_F(r) / 60.;
        const double x_motor = speed * dx / std::hypot(dx, dy);
        const bool   in_band = x_motor > band_lo && x_motor < band_hi;
        if (role == "Outer wall") {
            if (dx > dy)
                ++out.outer_along_x;
            else {
                ++out.outer_along_y;
                out.outer_along_y_min_speed = std::min(out.outer_along_y_min_speed, speed);
            }
            out.outer_x_in_band += in_band;
        } else {
            if (dx > dy)
                ++out.inner_along_x;
            out.inner_x_in_band += in_band;
        }
    });
    return out;
}

} // namespace

TEST_CASE("Motor resonance avoidance slows only in-scope walls whose stepper would run inside its band", "[MotorResonance]")
{
    // Walls at 60 mm/s on a Cartesian printer whose X stepper resonates at 50-70 mm/s. Walls along X
    // run the X stepper at 60 (inside); walls along Y do not turn it at all. Klipper flavour with a
    // different acceleration per wall puts SET_VELOCITY_LIMIT between each travel and its wall, which
    // must not make the avoider lose track of the position. The machine and volumetric limits are
    // raised so they cap neither the accelerations nor the speeds.
    const std::string scope = GENERATE("disabled", "outer_walls", "all_walls");
    CAPTURE(scope);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "gcode_flavor",               "klipper" },
        { "machine_max_acceleration_extruding", "20000,20000" },
        { "machine_max_acceleration_x", "20000,20000" },
        { "machine_max_acceleration_y", "20000,20000" },
        { "filament_max_volumetric_speed", "100" },
        { "default_acceleration",       5000 },
        { "outer_wall_acceleration",    2000 },
        { "inner_wall_acceleration",    4000 },
        { "motor_resonance_kinematics", "cartesian" },
        { "motor_resonance_bands_x",    "50-70" },
        { "motor_resonance_avoidance",  scope },
        { "outer_wall_speed",           60 },
        { "inner_wall_speed",           60 },
        { "wall_loops",                 2 },
        { "enable_arc_fitting",         "disabled" },
        { "slow_down_for_layer_cooling", false },
        { "enable_overhang_speed",      false },
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
    });
    const std::string gcode = slice({ TriangleMesh(its_make_cube(20, 20, 20)) }, config);
    CHECK(gcode.find("_MOTOR_RESONANCE") == std::string::npos);
    REQUIRE(gcode.find("SET_VELOCITY_LIMIT ACCEL=2000") != std::string::npos);

    const WallSpeeds s = wall_speeds(gcode, 50., 70.);
    REQUIRE(s.outer_along_x > 0);
    REQUIRE(s.outer_along_y > 0);
    REQUIRE(s.inner_along_x > 0);
    if (scope == "disabled") {
        CHECK(s.outer_x_in_band == s.outer_along_x);
        CHECK(s.inner_x_in_band == s.inner_along_x);
    } else {
        CHECK(s.outer_x_in_band == 0);
        CHECK(s.inner_x_in_band == (scope == "all_walls" ? 0 : s.inner_along_x));
    }
    // Moves along Y keep the speed the profile asked for.
    CHECK_THAT(s.outer_along_y_min_speed, Catch::Matchers::WithinAbs(60., 0.01));
}

TEST_CASE("Unreadable motor resonance ranges stop the print from validating", "[MotorResonance]")
{
    Print              print;
    Model              model;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({ { "motor_resonance_bands_y", "70-50" } });
    init_print({ TriangleMesh(its_make_cube(20, 20, 20)) }, print, model, config);
    const StringObjectException err = print.validate();
    CHECK(err.opt_key == "motor_resonance_bands_y");
    CHECK_FALSE(err.string.empty());
}

TEST_CASE("Avoider slows bracketed extrusions, restores the feedrate after them and drops its markers", "[MotorResonance]")
{
    MotorResonanceAvoider avoider(ShakeTune::MotorKinematics::CoreXY, { ShakeTune::SpeedBands{ { 55., 65. } }, ShakeTune::SpeedBands{} });
    const std::string     in = "G90\n"
                               "M83\n"
                               "G1 X0 Y0 F9000\n"
                               ";_MOTOR_RESONANCE_START\n"
                               "G1 F3600\n"
                               "G1 X10 Y10 E0.5 ; along motor A\n"
                               "G1 X20 Y0 E0.5\n"
                               "G1 X30 Y10 E0.5\n"
                               ";_MOTOR_RESONANCE_END\n"
                               "G1 X40 Y20 E0.5\n";
    const std::string     out = avoider.process_layer(std::string(in));
    CHECK(out.find("_MOTOR_RESONANCE") == std::string::npos);
    // 60 mm/s along motor A is inside 55-65: slowed to just under 55 mm/s = 3300 mm/min.
    CHECK(out.find("G1 X10 Y10 E0.5 F3299 ; along motor A\n") != std::string::npos);
    // Along motor B only: back to the speed the G-code asked for.
    CHECK(out.find("G1 X20 Y0 E0.5 F3600\n") != std::string::npos);
    CHECK(out.find("G1 X30 Y10 E0.5 F3299\n") != std::string::npos);
    // Outside the bracket the same move is left alone, and the feedrate restored.
    CHECK(out.find("G1 X40 Y20 E0.5 F3600\n") != std::string::npos);
}

TEST_CASE("Avoider leaves travel, retraction and unbracketed moves untouched", "[MotorResonance]")
{
    MotorResonanceAvoider avoider(ShakeTune::MotorKinematics::CoreXY, { ShakeTune::SpeedBands{ { 55., 65. } }, ShakeTune::SpeedBands{} });
    const std::string     in = "G90\nM83\nG1 X0 Y0 F3600\n"
                               ";_MOTOR_RESONANCE_START\n"
                               "G1 X10 Y10\n"
                               "G1 E-0.8 F2100\n"
                               ";_MOTOR_RESONANCE_END\n"
                               "G1 X20 Y20 E0.5 F3600\n";
    CHECK(avoider.process_layer(std::string(in)) == "G90\nM83\nG1 X0 Y0 F3600\nG1 X10 Y10\nG1 E-0.8 F2100\nG1 X20 Y20 E0.5 F3600\n");
}

TEST_CASE("Avoider forgets the position after a firmware macro", "[MotorResonance]")
{
    // After the macro the toolhead could be anywhere, so the next move's direction is unknown and
    // its speed is left alone; the absolute move after it pins the position down again.
    MotorResonanceAvoider avoider(ShakeTune::MotorKinematics::CoreXY, { ShakeTune::SpeedBands{ { 55., 65. } }, ShakeTune::SpeedBands{} });
    const std::string     out = avoider.process_layer("G90\nM83\nG1 X0 Y0 F3600\n"
                                                      ";_MOTOR_RESONANCE_START\n"
                                                      "TIMELAPSE_TAKE_FRAME\n"
                                                      "G1 X10 Y10 E0.5\n"
                                                      "G1 X20 Y20 E0.5\n"
                                                      ";_MOTOR_RESONANCE_END\n");
    CHECK(out.find("G1 X10 Y10 E0.5\n") != std::string::npos);
    CHECK(out.find("G1 X20 Y20 E0.5 F3299\n") != std::string::npos);
}

TEST_CASE("Avoider keeps the position across Klipper settings commands and tool changes", "[MotorResonance]")
{
    MotorResonanceAvoider avoider(ShakeTune::MotorKinematics::CoreXY, { ShakeTune::SpeedBands{ { 55., 65. } }, ShakeTune::SpeedBands{} });
    const std::string     out = avoider.process_layer("G90\nM83\nG1 X0 Y0 F3600\n"
                                                      "SET_VELOCITY_LIMIT ACCEL=3000\n"
                                                      "EXCLUDE_OBJECT_START NAME=cube\n"
                                                      "M204 S3000\n"
                                                      "T0\n"
                                                      ";_MOTOR_RESONANCE_START\n"
                                                      "G1 X10 Y10 E0.5\n"
                                                      ";_MOTOR_RESONANCE_END\n");
    CHECK(out.find("G1 X10 Y10 E0.5 F3299\n") != std::string::npos);
}
