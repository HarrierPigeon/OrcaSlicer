#include <catch2/catch_all.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/BeltGCodeWriter.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/PrintConfig.hpp"

using namespace Slic3r;

SCENARIO("set_speed emits values with fixed-point output.", "[GCodeWriter]") {

    GIVEN("GCodeWriter instance") {
        GCodeWriter writer;
        WHEN("set_speed is called to set speed to 99999.123") {
            THEN("Output string is G1 F99999.123") {
                REQUIRE_THAT(writer.set_speed(99999.123), Catch::Matchers::Equals("G1 F99999.123\n"));
            }
        }
        WHEN("set_speed is called to set speed to 1") {
            THEN("Output string is G1 F1") {
                REQUIRE_THAT(writer.set_speed(1.0), Catch::Matchers::Equals("G1 F1\n"));
            }
        }
        WHEN("set_speed is called to set speed to 203.200022") {
            THEN("Output string is G1 F203.2") {
                REQUIRE_THAT(writer.set_speed(203.200022), Catch::Matchers::Equals("G1 F203.2\n"));
            }
        }
        WHEN("set_speed is called to set speed to 203.200522") {
            THEN("Output string is G1 F203.201") {
                REQUIRE_THAT(writer.set_speed(203.200522), Catch::Matchers::Equals("G1 F203.201\n"));
            }
        }
    }
}

SCENARIO("z_hop lifts the nozzle when a lift is requested", "[GCodeWriter]") {
    GIVEN("A writer with the nozzle parked at Z = 10") {
        GCodeWriter writer;
        std::vector<unsigned int> extruder_ids { 0 };
        writer.set_extruders(extruder_ids);
        writer.set_extruder(0);
        writer.travel_to_z(10.0);

        WHEN("z_hop is 1 and an eager lift is requested") {
            writer.config.z_hop.values = { 1.0 };
            std::string gcode = writer.eager_lift(LiftType::NormalLift);
            THEN("a Z move up by z_hop is emitted") {
                REQUIRE_THAT(gcode, Catch::Matchers::ContainsSubstring("Z11"));
            }
        }
        WHEN("z_hop is 0") {
            writer.config.z_hop.values = { 0.0 };
            std::string gcode = writer.eager_lift(LiftType::NormalLift);
            THEN("no lift is emitted") {
                REQUIRE(gcode.empty());
            }
        }
    }
}

// Regression test for the belt-printer "illegal gantry move at print start" bug.
//
// On a belt printer the layer-change z-hop is deferred (lazy_lift) and consumed
// by the first travel_to_xyz, whose NormalLift branch lifts in place via
// _travel_to_z(). On a normal printer _travel_to_z emits a Z-only move, but in
// belt mode Z is coupled to Y/X, so _travel_to_z re-emits the current m_pos
// through the belt shear. At print start (and after custom gcode)
// is_current_position_clear() is false and m_pos.xy is still the uninitialised
// origin (0,0), which shears into machine (X=bed_max, Y=layer_z) — a move far up
// the gantry, e.g. "G1 X95 Y168.19 Z237.857". The fix guards that lift on
// is_current_position_clear(), mirroring the SlopeLift branch.
SCENARIO("Belt: the first travel does not lift through the uninitialised origin", "[GCodeWriter][belt]")
{
    GIVEN("A fresh BeltGCodeWriter configured for an X-tilt 45 degree belt") {
        // Machine-frame + slicer->world back-transform config (X tilt, 45 deg).
        PrintConfig belt_config;
        belt_config.belt_printer.value               = true;
        belt_config.gcode_back_transform.value       = true;
        belt_config.belt_slice_rotation.value        = BeltRotationAxis::X;
        belt_config.belt_slice_rotation_angle.value  = 45.0;
        belt_config.belt_slice_rotation_global.value = true;
        belt_config.belt_preslice_global.value       = true;
        belt_config.belt_frame_tilt_decouple.value   = false;
        belt_config.belt_frame_tilt_angle.value      = 45.0;

        BeltGCodeWriter writer;
        writer.set_machine_frame_transform(belt_config);
        writer.set_belt_back_transform(belt_config);

        std::vector<unsigned int> extruder_ids { 0 };
        writer.set_extruders(extruder_ids);
        writer.set_extruder(0);
        writer.config.travel_speed.value        = 100.0;
        writer.config.z_hop.values              = { 0.4 };
        writer.config.retract_lift_above.values = { 0.0 };
        writer.config.retract_lift_below.values = { 0.0 };

        // A fresh writer has not established its planar position yet — this is the
        // precondition that made the origin leak into the first move.
        REQUIRE_FALSE(writer.is_current_position_clear());

        WHEN("a layer-change z-hop is pending and we travel to the first object point") {
            // Defer a z-hop, exactly as a retract on layer change leaves it.
            writer.lazy_lift(LiftType::NormalLift);

            // First object point in slicing coordinates: a near-belt point (y ~= -z)
            // so its transformed gantry Y is small (~1mm). The bogus origin lift, in
            // contrast, would shear to machine Y ~= nominal_z.
            const double nominal_z = 100.0;
            std::string gcode = writer.travel_to_xyz(Vec3d(10.0, -(nominal_z - 1.0), nominal_z));

            THEN("no emitted move flies up the gantry; machine Y stays near the part") {
                double max_y = std::numeric_limits<double>::lowest();
                GCodeReader reader;
                reader.parse_buffer(gcode, [&max_y](GCodeReader &, const GCodeReader::GCodeLine &line) {
                    if (line.cmd_is("G1") && line.has(Y))
                        max_y = std::max(max_y, double(line.y()));
                });
                // The destination shears to machine Y ~= 1mm. The old origin-lift bug
                // produced a separate move at machine Y ~= nominal_z (100mm), so any
                // Y well above the part means the origin leaked into a move.
                REQUIRE(max_y > 0.0);   // the destination move was emitted and parsed
                REQUIRE(max_y < 10.0);  // ... and nothing flew up the gantry
            }
        }
    }
}
