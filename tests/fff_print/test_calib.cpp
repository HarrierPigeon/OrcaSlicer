#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Format/DRC.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/calib.hpp"
#include "libslic3r/libslic3r.h"
#include "test_helpers.hpp"

using namespace Slic3r;

namespace {

// A move of the PA line test and the pressure advance it runs with.
struct Move
{
    std::string type; // the ";TYPE:" tag it is under
    size_t      tag;  // which ";TYPE:" line that is
    Vec2d       from, to;
    double      feedrate; // mm/min
    double      e;        // relative extrusion
    double      pa;       // ADVANCE of the last SET_PRESSURE_ADVANCE before it

    double length() const { return (to - from).norm(); }
    bool   extrudes_xy() const { return e > 0. && length() > 0.; }
    double e_per_mm() const { return e / length(); }
};

// Every G0/G1 that moves in XY or E.
std::vector<Move> moves(const std::string &gcode)
{
    std::vector<Move>  out;
    std::istringstream lines(gcode);
    std::string        line, type;
    size_t             tag      = 0;
    bool               relative = false;
    Vec2d              pos      = Vec2d::Zero();
    double             e = 0., f = 0., pa = 0.;
    while (std::getline(lines, line)) {
        if (line.rfind(";TYPE:", 0) == 0) {
            type = line.substr(6);
            ++tag;
            continue;
        }
        if (line.rfind("SET_PRESSURE_ADVANCE ", 0) == 0) {
            const size_t value = line.find(" ADVANCE=");
            if (value != std::string::npos)
                pa = std::stod(line.substr(value + 9));
            continue;
        }
        if (line.rfind("M83", 0) == 0)
            relative = true;
        else if (line.rfind("M82", 0) == 0)
            relative = false;
        const bool is_move = line.rfind("G1 ", 0) == 0 || line.rfind("G0 ", 0) == 0;
        if (!is_move && line.rfind("G92 ", 0) != 0)
            continue;

        std::istringstream words(line.substr(0, line.find(';')));
        std::string        word;
        words >> word; // the command
        Vec2d  to = pos;
        double ev = 0.;
        bool   has_e = false;
        while (words >> word) {
            const double v = std::stod(word.substr(1));
            switch (word[0]) {
            case 'X': to.x() = v; break;
            case 'Y': to.y() = v; break;
            case 'E': ev = v; has_e = true; break;
            case 'F': f = v; break;
            default: break;
            }
        }
        if (!is_move) { // G92
            if (has_e)
                e = ev;
            continue;
        }
        if (has_e || to != pos)
            out.push_back({type, tag, pos, to, f, has_e ? (relative ? ev : ev - e) : 0., pa});
        if (has_e)
            e = relative ? e + ev : ev;
        pos = to;
    }
    return out;
}

// Extruding moves under a ";TYPE:<type>" tag: only the first one after each tag, or all of them. The PA line test prints
// no object, so its fast segments are the first move after each ";TYPE:Outer wall" and the box under its numbers is
// all of ";TYPE:Bottom surface".
std::vector<Move> extrusions_of_type(const std::string &gcode, const std::string &type, bool first_only)
{
    std::vector<Move> out;
    for (const Move &move : moves(gcode))
        if (move.type == type && move.extrudes_xy() && !(first_only && !out.empty() && out.back().tag == move.tag))
            out.push_back(move);
    return out;
}

} // namespace

TEST_CASE("The PA line test prints with the speed and extrusion of the filament assigned to it", "[Calib]")
{
    // Filament 1: 1.75 mm, flow ratio 1, up to 21 mm3/s. Filament 2: 2.85 mm, flow ratio 0.9, up to 4 mm3/s, so its
    // volumetric limit caps the fast lines well below filament 1's.
    const int  filament      = GENERATE(1, 2);
    const bool print_numbers = GENERATE(false, true);
    CAPTURE(filament, print_numbers);
    DynamicPrintConfig config = Test::multifilament_config(2, {
        {"filament_self_index", "1,2"},
        {"filament_extruder_variant", "Direct Drive Standard;Direct Drive Standard"},
        {"filament_diameter", "1.75,2.85"},
        {"filament_flow_ratio", "1,0.9"},
        {"filament_max_volumetric_speed", "21,4"},
        {"print_flow_ratio", "1"},
        {"outer_wall_speed", "200"},
        {"initial_layer_print_height", "0.2"},
        {"enable_prime_tower", 0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
    });
    Print print;
    Model model;
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{{{"extruder", std::to_string(filament)}}};
    Test::init_print({Test::cube(20)}, print, model, config, &overrides);
    Calib_Params params;
    params.mode          = CalibMode::Calib_PA_Line;
    params.start         = 0.;
    params.end           = 0.02;
    params.step          = 0.002;
    params.print_numbers = print_numbers;
    print.set_calib_params(params);
    const std::string       gcode    = Test::gcode(print);
    const std::vector<Move> segments = extrusions_of_type(gcode, "Outer wall", true);
    REQUIRE_FALSE(segments.empty());
    // The box under the numbers is drawn with the same line width and height as the PA lines.
    const std::vector<Move> box = extrusions_of_type(gcode, "Bottom surface", false);
    CHECK(box.empty() != print_numbers);

    // The test prints 1.5 nozzle-wide lines on the 0.2 mm first layer (CalibPressureAdvanceLine).
    const size_t idx          = size_t(filament - 1);
    const double mm3_per_mm   = Flow(0.4 * 1.5, 0.2, 0.4).mm3_per_mm();
    const double max_vol      = config.option<ConfigOptionFloats>("filament_max_volumetric_speed")->get_at(idx);
    const double diameter     = config.option<ConfigOptionFloats>("filament_diameter")->get_at(idx);
    const double flow_ratio   = config.option<ConfigOptionFloatsNullable>("filament_flow_ratio")->get_at(idx);
    // Fast lines run at the outer wall speed (at least 100 mm/s), capped by the filament's volumetric speed.
    const double fast_speed   = std::floor(std::min(std::max(100., 200.), max_vol / mm3_per_mm));
    const double e_per_mm     = mm3_per_mm * flow_ratio / (PI * diameter * diameter / 4.);
    for (const Move &move : segments) {
        CHECK_THAT(move.feedrate, Catch::Matchers::WithinAbs(fast_speed * 60., 1.));
        CHECK_THAT(move.e_per_mm(), Catch::Matchers::WithinRel(e_per_mm, 0.005));
    }
    for (const Move &move : box)
        CHECK_THAT(move.e_per_mm(), Catch::Matchers::WithinRel(e_per_mm, 0.005));
}

namespace {

// A 0.4 mm nozzle on a 200 mm bed, with machine limits above the accelerations the tests use. The PA line test prints
// its fast segments at the outer wall speed, which the filament's volumetric speed leaves at 250 mm/s.
DynamicPrintConfig split_pa_config(const std::string &flavor, double outer_wall_acceleration)
{
    DynamicPrintConfig config = Test::multifilament_config(1, {
        {"gcode_flavor", flavor},
        {"outer_wall_acceleration", outer_wall_acceleration},
        {"outer_wall_speed", "250"},
        {"filament_max_volumetric_speed", "40"},
        {"machine_max_acceleration_extruding", "20000,20000"},
        {"machine_max_acceleration_x", "20000,20000"},
        {"machine_max_acceleration_y", "20000,20000"},
        {"initial_layer_print_height", "0.2"},
        {"enable_prime_tower", 0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
    });
    return config;
}

} // namespace

TEST_CASE("The split PA line test switches pressure advance between two collinear halves of each fast segment", "[Calib]")
{
    const CalibPASplit vary = GENERATE(CalibPASplit::VarySpeedUp, CalibPASplit::VarySlowDown);
    // At 500 mm/s2 the segment 250 mm/s needs does not fit the bed and the fast speed comes down, at 1500 mm/s2 Klipper's
    // minimum cruise ratio sets its length, and at 10000 mm/s2 the smoothing window does.
    const double accel = GENERATE(500., 1500., 10000.);
    CAPTURE(vary == CalibPASplit::VarySpeedUp, accel);
    Print print;
    Model model;
    Test::init_print({Test::cube(20)}, print, model, split_pa_config("klipper", accel));
    Calib_Params params;
    params.mode           = CalibMode::Calib_PA_Line;
    params.start          = 0.012;
    params.end            = 0.028;
    params.step           = 0.004;
    params.pa_split       = vary;
    params.pa_split_fixed = 0.02;
    print.set_calib_params(params);
    REQUIRE(print.validate().string.empty());
    const std::vector<Move> all = moves(Test::gcode(print));

    // Each line is a slow segment, the two halves of the fast segment under one ";TYPE:Outer wall", and a slow segment.
    std::vector<size_t> first_halves;
    for (size_t i = 1; i < all.size(); ++i)
        if (all[i].type == "Outer wall" && all[i - 1].tag != all[i].tag)
            first_halves.push_back(i);
    const int lines = int(std::lround((params.end - params.start) / params.step)) + 1; // 0.012 to 0.028 is 5 lines
    REQUIRE(first_halves.size() == size_t(lines));

    for (int line = 0; line < lines; ++line) {
        CAPTURE(line);
        const double varied  = params.start + line * params.step;
        const double pa_up   = vary == CalibPASplit::VarySpeedUp ? varied : params.pa_split_fixed;
        const double pa_down = vary == CalibPASplit::VarySlowDown ? varied : params.pa_split_fixed;
        const size_t i       = first_halves[line];
        REQUIRE(i + 2 < all.size());
        const Move &slow_in = all[i - 1], &first = all[i], &second = all[i + 1], &slow_out = all[i + 2];

        // Nothing moves between the halves, which run along one line at one speed and flow.
        REQUIRE(second.tag == first.tag);
        REQUIRE(second.extrudes_xy());
        CHECK_THAT((second.from - first.to).norm(), Catch::Matchers::WithinAbs(0., 1e-6));
        CHECK_THAT((first.to - first.from).normalized().dot((second.to - second.from).normalized()), Catch::Matchers::WithinAbs(1., 1e-9));
        CHECK_THAT(second.feedrate, Catch::Matchers::WithinAbs(first.feedrate, 1e-9));
        CHECK_THAT(second.e_per_mm(), Catch::Matchers::WithinRel(first.e_per_mm(), 1e-6));

        // The speed-up PA is set before the line and the slow-down PA at the joint, so Klipper applies it from the second half.
        CHECK_THAT(slow_in.pa, Catch::Matchers::WithinAbs(pa_up, 1e-6));
        CHECK_THAT(first.pa, Catch::Matchers::WithinAbs(pa_up, 1e-6));
        CHECK_THAT(second.pa, Catch::Matchers::WithinAbs(pa_down, 1e-6));
        CHECK_THAT(slow_out.pa, Catch::Matchers::WithinAbs(pa_down, 1e-6));

        // The joint is in the middle of the cruise. Klipper ramps between the slow and the fast speed at the start of the
        // first half and at the end of the second, and smooths each move's PA over a window of 0.04 s by default, centred:
        // the windows of the ramps and of the joint stay apart when each half cruises for at least that long.
        CHECK_THAT(second.length(), Catch::Matchers::WithinAbs(first.length(), 1e-6));
        const double fast = first.feedrate / 60., slow = slow_in.feedrate / 60.;
        const double ramp = (fast * fast - slow * slow) / (2. * accel);
        // The G-code gives positions to the micrometre, so a half sized exactly to a bound may come out that much short.
        const double gcode_resolution = 1e-3;
        for (const Move *half : {&first, &second}) {
            CHECK(half->length() - ramp >= 0.04 * fast - gcode_resolution);
            // Klipper's default minimum_cruise_ratio of 0.5 lets the segment reach the fast speed only when it cruises
            // for half its length.
            CHECK(half->length() >= 2. * ramp - gcode_resolution);
        }
    }
}

TEST_CASE("The split PA line test needs Klipper and pressure advance values of 0 or more", "[Calib]")
{
    // Marlin finishes its planned moves before it changes the K factor (M900 synchronizes the planner).
    const std::string flavor = GENERATE(std::string("klipper"), std::string("marlin2"));
    const double      start  = GENERATE(-0.01, 0., 0.01);
    const double      fixed  = GENERATE(-0.02, 0., 0.02);
    CAPTURE(flavor, start, fixed);
    Print print;
    Model model;
    Test::init_print({Test::cube(20)}, print, model, split_pa_config(flavor, 3000.));
    Calib_Params params;
    params.mode           = CalibMode::Calib_PA_Line;
    params.start          = start;
    params.end            = 0.05;
    params.step           = 0.01;
    params.pa_split       = CalibPASplit::VarySpeedUp;
    params.pa_split_fixed = fixed;
    print.set_calib_params(params);
    CHECK(print.validate().string.empty() == (flavor == "klipper" && start >= 0. && fixed >= 0.));
}

TEST_CASE("The split PA line test prints a pressure advance of 0 as 0.0001, so that Klipper does not stop at the switch", "[Calib]")
{
    // Klipper stops the toolhead to turn pressure advance on or off, which would stop each line at its PA switch.
    const CalibPASplit vary = GENERATE(CalibPASplit::VarySpeedUp, CalibPASplit::VarySlowDown);
    CAPTURE(vary == CalibPASplit::VarySpeedUp);
    Print print;
    Model model;
    Test::init_print({Test::cube(20)}, print, model, split_pa_config("klipper", 3000.));
    Calib_Params params;
    params.mode           = CalibMode::Calib_PA_Line;
    params.start          = 0.;
    params.end            = 0.02;
    params.step           = 0.01;
    params.pa_split       = vary;
    params.pa_split_fixed = 0.;
    print.set_calib_params(params);
    REQUIRE(print.validate().string.empty());
    const std::vector<Move> all = moves(Test::gcode(print));

    // From the slow segment before each fast one to the slow segment after it, pressure advance stays on.
    size_t lines = 0;
    for (size_t i = 1; i + 2 < all.size(); ++i)
        if (all[i].type == "Outer wall" && all[i - 1].tag != all[i].tag) {
            CAPTURE(lines);
            const double varied = params.start + lines * params.step;
            for (const Move *move : {&all[i - 1], &all[i], &all[i + 1], &all[i + 2]})
                CHECK(move->pa > 0.);
            const Move &first = all[i], &second = all[i + 1];
            CHECK_THAT(vary == CalibPASplit::VarySpeedUp ? first.pa : second.pa, Catch::Matchers::WithinAbs(std::max(varied, 1e-4), 1e-9));
            CHECK_THAT(vary == CalibPASplit::VarySpeedUp ? second.pa : first.pa, Catch::Matchers::WithinAbs(1e-4, 1e-9));
            ++lines;
        }
    CHECK(lines == 3);
}

TEST_CASE("The split PA line test has an outer wall with tick marks at the speed-up, the PA switch and the slow-down", "[Calib]")
{
    const bool numbers = GENERATE(false, true);
    CAPTURE(numbers);
    Print print;
    Model model;
    Test::init_print({Test::cube(20)}, print, model, split_pa_config("klipper", 3000.));
    Calib_Params params;
    params.mode           = CalibMode::Calib_PA_Line;
    params.start          = 0.012;
    params.end            = 0.028;
    params.step           = 0.004;
    params.print_numbers  = numbers;
    params.pa_split       = CalibPASplit::VarySlowDown;
    params.pa_split_fixed = 0.02;
    print.set_calib_params(params);
    const std::vector<Move> all = moves(Test::gcode(print));

    // The two halves of each line's fast segment.
    const std::vector<Move> halves = extrusions_of_type(Test::gcode(print), "Outer wall", false);
    REQUIRE(halves.size() == 10);
    const double first_y = halves.front().from.y(), last_y = halves.back().from.y();
    const double speed_up_x = halves.front().from.x(), switch_x = halves.front().to.x(), slow_down_x = halves[1].to.x();
    const double anchor_x   = speed_up_x - 20.; // the slow segments are 20 mm long and start at the anchor line

    std::vector<Move> extrusions;
    BoundingBoxf      extents;
    for (const Move &move : all)
        if (move.extrudes_xy()) {
            extrusions.push_back(move);
            extents.merge(move.from);
            extents.merge(move.to);
        }
    const auto extruded = [&extrusions](const Vec2d &from, const Vec2d &to) {
        return std::any_of(extrusions.begin(), extrusions.end(),
                           [&](const Move &move) { return (move.from - from).norm() < 1e-3 && (move.to - to).norm() < 1e-3; });
    };

    // The wall runs around everything the test extrudes, numbers included.
    const Vec2d min = extents.min, max = extents.max;
    CHECK(extruded(min, Vec2d(max.x(), min.y())));
    CHECK(extruded(Vec2d(max.x(), min.y()), max));
    CHECK(extruded(max, Vec2d(min.x(), max.y())));
    CHECK(extruded(Vec2d(min.x(), max.y()), min));
    // It touches the anchor line on the left, which the lines start from, and runs clear of the first and last line.
    CHECK(anchor_x - min.x() > 0.);
    CHECK(anchor_x - min.x() <= 0.6);
    CHECK(first_y - min.y() > 3.);
    CHECK(max.y() - last_y > 3.);

    // At each transition a tick reaches from the wall towards the first line and another towards the last one. Each
    // stops clear of the line, which is 0.6 mm wide for this 0.4 mm nozzle.
    for (const double x : {speed_up_x, switch_x, slow_down_x}) {
        CAPTURE(x);
        const auto ticks = [&](double wall_y, double line_y) {
            return std::count_if(extrusions.begin(), extrusions.end(), [&](const Move &move) {
                return std::abs(move.from.x() - x) < 1e-6 && std::abs(move.to.x() - x) < 1e-6 && std::abs(move.from.y() - wall_y) < 1e-6 &&
                       std::abs(move.to.y() - wall_y) < std::abs(line_y - wall_y) && std::abs(move.to.y() - line_y) > 1.;
            });
        };
        CHECK(ticks(min.y(), first_y) == 1);
        CHECK(ticks(max.y(), last_y) == 1);
    }
}

TEST_CASE("Every VFA test model loads as a closed tower of its number of speed blocks", "[Calib]")
{
    const VfaTestModel model = GENERATE(from_range(std::begin(vfa_test_models), std::end(vfa_test_models)));
    CAPTURE(model.file);
    TriangleMesh mesh;
    REQUIRE(load_drc((std::string(PROFILES_DIR) + "/../calib/vfa/" + model.file).c_str(), &mesh));
    CHECK(its_num_open_edges(mesh.its) == 0);
    CHECK_THAT(mesh.bounding_box().size().z(), Catch::Matchers::WithinAbs(model.blocks * vfa_base_block_height, 0.01));
}
