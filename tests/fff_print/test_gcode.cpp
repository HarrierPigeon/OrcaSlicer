#include <catch2/catch_all.hpp>

#include <cmath>
#include <memory>

#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"

using namespace Slic3r;

SCENARIO("Origin manipulation", "[GCode]") {
	Slic3r::GCode gcodegen;
	WHEN("set_origin to (10,0)") {
    	gcodegen.set_origin(Vec2d(10,0));
    	REQUIRE(gcodegen.origin() == Vec2d(10, 0));
    }
	WHEN("set_origin to (10,0) and translate by (5, 5)") {
		gcodegen.set_origin(Vec2d(10,0));
		gcodegen.set_origin(gcodegen.origin() + Vec2d(5, 5));
		THEN("origin returns reference to point") {
    		REQUIRE(gcodegen.origin() == Vec2d(15,5));
    	}
    }
}

// Regression test for the belt-printer "phantom extrusion line from Y=0" bug.
//
// GCodeProcessor::store_move_vertex pins a move's stored Z to the first-layer
// height while m_processing_start_custom_gcode is set (the start G-code "prepare"
// stage), because on a normal printer the toolhead Z there is not yet a real print
// height. On a belt printer that override is wrong: Z is written explicitly and the
// designed-view back-transform couples machine Z into the rendered model Y (the
// belt tilt mixes the height and belt-feed axes). Overriding it back-transforms the
// last prepare-stage move (the unretract right before the first extrusion) to
// model Y ~= 0, and libvgcode then draws a phantom extrusion segment from Y ~= 0 to
// the first real toolpath — rendered in the first extrusion role's color. The fix
// keeps the real Z for belt printers (gated on belt_tilt_angle). Here we assert the
// prepare-stage move keeps its real Z so it can no longer leak to Y ~= 0.
SCENARIO("Belt: start-gcode prepare-stage moves keep their real Z", "[GCode][belt]")
{
    // Belt printers are non-Bambu, so the G-code uses the "compatible" reserved
    // tags ("TYPE:" for the extrusion role). The processor selects the tag table
    // from the static s_IsBBLPrinter flag, so mirror the belt-printer setting here
    // (saved/restored so test ordering stays unaffected).
    struct BBLPrinterGuard {
        bool prev = GCodeProcessor::s_IsBBLPrinter;
        BBLPrinterGuard()  { GCodeProcessor::s_IsBBLPrinter = false; }
        ~BBLPrinterGuard() { GCodeProcessor::s_IsBBLPrinter = prev; }
    } bbl_guard;

    GIVEN("A belt G-code whose start sequence travels to a high machine Z before the first extrusion") {
        // The leading "; belt_slice_rotation_angle = 45" header sets belt_tilt_angle
        // (parsed before the body), enabling the belt code path. ;TYPE:Custom before
        // any G1 turns on the prepare stage; ;TYPE:Outer wall turns it off, exactly
        // as a sliced belt print is laid out.
        const std::string gcode =
            "; belt_slice_rotation_angle = 45\n"
            "G90\n"
            "G21\n"
            "M83\n"
            ";TYPE:Custom\n"
            "G1 E-1.5 F2100\n"            // retract at the (0,0,0) origin
            "G1 X45 Y0.3 Z50 F12000\n"   // travel to the approach point (prepare stage)
            "G1 E1.5 F1800\n"            // unretract in place (prepare stage)
            ";TYPE:Outer wall\n"
            "G1 X46 Y0.3 Z50 E0.05\n";   // first extrusion, same Z as the approach

        GCodeProcessor processor;
        processor.process_buffer(gcode);
        const GCodeProcessorResult& result = processor.get_result();

        THEN("the belt code path is active") {
            REQUIRE_THAT(result.belt_tilt_angle, Catch::Matchers::WithinAbs(45.0, 1e-4));
        }

        WHEN("locating the first extrusion and the move that precedes it") {
            size_t first_extrude = result.moves.size();
            for (size_t i = 0; i < result.moves.size(); ++i)
                if (result.moves[i].type == EMoveType::Extrude) { first_extrude = i; break; }

            THEN("an extrusion and a preceding move exist") {
                REQUIRE(first_extrude < result.moves.size());
                REQUIRE(first_extrude > 0);
            }

            THEN("the preceding prepare-stage move shares the extrusion's real Z (no leak to Y=0)") {
                const float extrude_z = result.moves[first_extrude].position.z();
                const float prev_z    = result.moves[first_extrude - 1].position.z();
                // The first extrusion is at the real Z=50; before the fix the
                // prepare-stage move's Z was pinned to the first-layer height
                // (0 here) instead, which back-transforms to model Y ~= 0 and
                // produces the phantom extrusion segment.
                REQUIRE_THAT(extrude_z, Catch::Matchers::WithinAbs(50.0, 1e-3));
                REQUIRE_THAT(prev_z,    Catch::Matchers::WithinAbs(50.0, 1e-3));
            }
        }
    }
}
