#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <math.h>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <boost/algorithm/string/predicate.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/GCode/SpiralVase.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

#include "test_helpers.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

// The extrusion moves of `gcode`.
std::vector<GCodeReader::GCodeLine> extrusions(const PrintConfig &config, const std::string &gcode)
{
    std::vector<GCodeReader::GCodeLine> out;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&out](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self))
            out.push_back(line);
    });
    return out;
}

} // namespace

TEST_CASE("Spiral vase keeps the selected coordinate and extrusion precision", "[SpiralVase][Regression]")
{
    const bool high_precision = GENERATE(false, true);
    CAPTURE(high_precision);
    PrintConfig config;
    config.high_precision_gcode.value        = high_precision;
    config.use_relative_e_distances.value    = true;
    config.spiral_mode_smooth.value          = false;
    config.spiral_starting_flow_ratio.value  = 0.5;
    config.spiral_finishing_flow_ratio.value = 0.5;
    config.resolution.value                  = 0.001;

    SpiralVase vase(config);
    vase.process_layer("G1 X0 Y0 Z0.2\n", false);

    // The first spiral layer ramps Z up from 0.2 and the flow up from half. A third of the way along,
    // Z is 0.2 + 0.02 / 3 and E is 0.001234 * (0.5 + 0.5 / 3).
    vase.enable(true);
    const std::vector<GCodeReader::GCodeLine> first = extrusions(config,
        vase.process_layer("G1 Z0.22\nG1 X1 Y0 E0.001234 F3600\nG1 X1 Y2 E0.002468\n", false));
    REQUIRE(first.size() == 2);
    CHECK_THAT(first.front().precise_value(Z), Catch::Matchers::WithinAbs(high_precision ? 0.2067 : 0.207, 1e-9));
    CHECK_THAT(first.front().precise_value(E), Catch::Matchers::WithinAbs(high_precision ? 0.000823 : 0.00082, 1e-9));

    // The last layer is followed by a copy of its moves whose flow ramps down to half.
    // Halfway along, E is 0.001236 * (0.5 + 0.5 * 0.5).
    vase.enable(true);
    const std::vector<GCodeReader::GCodeLine> last = extrusions(config,
        vase.process_layer("G1 Z0.24\nG1 X1 Y0 E0.001236\nG1 X1 Y2 E0.002468\n", true));
    REQUIRE(last.size() == 4);
    CHECK_THAT(last[2].precise_value(E), Catch::Matchers::WithinAbs(high_precision ? 0.000927 : 0.00093, 1e-9));
}

TEST_CASE("Spiral vase mode warns about an object that overrides infill, walls or top shells", "[SpiralVase]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "spiral_mode",           1 },
        { "sparse_infill_density", "0%" },
        { "wall_loops",            1 },
        { "top_shell_layers",      0 },
        { "layer_change_gcode",    "G92 E0\n" }, // validate() relative-E reset
    });
    const auto [opt_key, value] = GENERATE(table<std::string, std::string>({
        { "",                      "" }, // no override: the vase-ready global config applies
        { "sparse_infill_density", "13%" },
        { "wall_loops",            "2" },
        { "top_shell_layers",      "3" },
    }));
    CAPTURE(opt_key);
    std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides(1);
    if (!opt_key.empty())
        overrides.front().emplace_back(opt_key, value);

    Print print;
    Model model;
    init_print({cube(20)}, print, model, config, &overrides);

    std::vector<StringObjectException> warnings;
    REQUIRE(print.validate(&warnings).string.empty());
    const PrintObject *object = print.objects().front();
    const auto warning = std::find_if(warnings.begin(), warnings.end(),
                                      [object](const StringObjectException &w) { return w.object == object; });
    if (opt_key.empty()) {
        CHECK(warning == warnings.end());
    } else {
        REQUIRE(warning != warnings.end());
        CHECK(warning->opt_key == opt_key);
    }
}

// Belt printers keep their supports under spiral vase: the counter-rotated calibration
// objects stand on a support wedge (Plater::_calib_apply_belt_mode), and a layer that
// carries support never spiralizes anyway (GCode::process_layer).
TEST_CASE("Spiral vase mode accepts supports on a belt printer", "[SpiralVase][belt][Support]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "belt_printer",               1 },
        { "belt_slice_rotation",        "x" },
        { "belt_slice_rotation_angle",  45 },
        { "gcode_remap_x",              "rev_x" },
        { "gcode_remap_y",              "pos_z" },
        { "gcode_remap_z",              "pos_y" },
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "skirt_loops",                0 },
        { "brim_type",                  "no_brim" },
        { "spiral_mode",                1 },
        { "sparse_infill_density",      "0%" },
        { "wall_loops",                 1 },
        { "top_shell_layers",           0 },
        { "bottom_shell_layers",        0 },
        { "enable_support",             1 },
        { "support_type",               "tree(auto)" },
        { "support_style",              "tree_hybrid" },
        { "support_base_pattern",       "rectilinear" },
        { "machine_start_gcode",        "T[initial_tool]\n" },
        { "layer_change_gcode",         "G92 E0\n" },
    });

    SECTION("The CLI config check lets the combination through on a belt only")
    {
        // Accepted means no error at all, for enabled and for enforced supports alike.
        CHECK(config.validate(/*under_cli=*/true).empty());
        DynamicPrintConfig enforced = config;
        enforced.set_deserialize_strict({ { "enable_support", 0 }, { "enforce_support_layers", 3 } });
        CHECK(enforced.validate(/*under_cli=*/true).empty());
        DynamicPrintConfig flat = config;
        flat.set_deserialize_strict({ { "belt_printer", 0 } });
        CHECK(flat.validate(/*under_cli=*/true).count("enable_support") == 1);
        DynamicPrintConfig flat_enforced = enforced;
        flat_enforced.set_deserialize_strict({ { "belt_printer", 0 } });
        CHECK(flat_enforced.validate(/*under_cli=*/true).count("enforce_support_layers") == 1);
    }

    SECTION("A counter-rotated cube slices with its support wedge")
    {
        // The calibration pose: the cube's bottom face leans over the belt, so the wedge
        // between that face and the belt is support printed before the object.
        indexed_triangle_set its = its_make_cube(20., 20., 20.);
        its_transform(its, Transform3d(Eigen::AngleAxisd(-M_PI / 4., Vec3d::UnitX())));
        Print print;
        Model model;
        init_print({ TriangleMesh(std::move(its)) }, print, model, config);
        // On the bed, not at its corner: the wedge is clipped to the bed outline.
        ModelInstance &instance = *model.objects.front()->instances.front();
        instance.set_offset(Vec3d(100., 100., instance.get_offset().z()));
        print.apply(model, config);

        std::vector<StringObjectException> warnings;
        REQUIRE(print.validate(&warnings).string.empty());
        const std::string gcode = Test::gcode(print);
        const PrintObject &object = *print.objects().front();
        CHECK(std::any_of(object.support_layers().begin(), object.support_layers().end(),
                          [](const SupportLayer *layer) { return ! layer->support_fills.empty(); }));
        CHECK(gcode.find(";TYPE:Support") != std::string::npos);
        CHECK(gcode.find(";TYPE:Outer wall") != std::string::npos);
    }

    SECTION("A brim under the wedge's feet is accepted beside spiral vase")
    {
        config.set_deserialize_strict({
            { "brim_type",    "outer_only" },
            { "brim_width",   3 },
            { "support_brim", 1 },
        });
        indexed_triangle_set its = its_make_cube(20., 20., 20.);
        its_transform(its, Transform3d(Eigen::AngleAxisd(-M_PI / 4., Vec3d::UnitX())));
        Print print;
        Model model;
        init_print({ TriangleMesh(std::move(its)) }, print, model, config);
        ModelInstance &instance = *model.objects.front()->instances.front();
        instance.set_offset(Vec3d(100., 100., instance.get_offset().z()));
        print.apply(model, config);

        std::vector<StringObjectException> warnings;
        REQUIRE(print.validate(&warnings).string.empty());
        const PrintObject &object = *print.objects().front();
        REQUIRE(object.has_belt_brim());
        const std::string gcode = Test::gcode(print);
        CHECK(gcode.find(";TYPE:Brim") != std::string::npos);
        CHECK(gcode.find(";TYPE:Support") != std::string::npos);
        CHECK(gcode.find(";TYPE:Outer wall") != std::string::npos);
        // The wedge shares the cube's layers here, so its feet's bands ride on them.
        const std::vector<ExtrusionEntityCollection> &bands = object.belt_brim_by_layer();
        CHECK(std::any_of(bands.begin(), bands.end(), [](const ExtrusionEntityCollection &b) { return ! b.empty(); }));
    }
}

// On a tilted belt the brim is laid band by band over many layers.  A layer that carries
// one of its bands is printed flat, the way layer 0 is with a brim on a flat bed, and the
// spiral starts once the part has left the belt (GCode::process_layer).  Under the IR3
// remap the belt axis is the G-code's Z: a flat layer holds one Z over all its extrusions,
// a spiral layer ramps it by the layer's pitch along the belt.
TEST_CASE("Belt brim layers print flat beside spiral vase and the spiral starts above them", "[SpiralVase][belt][Brim]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "belt_printer",               1 },
        { "belt_slice_rotation",        "x" },
        { "belt_slice_rotation_angle",  45 },
        { "gcode_remap_x",              "rev_x" },
        { "gcode_remap_y",              "pos_z" },
        { "gcode_remap_z",              "pos_y" },
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "skirt_loops",                0 },
        { "brim_type",                  "outer_only" },
        { "brim_width",                 3 },
        { "spiral_mode",                1 },
        { "sparse_infill_density",      "0%" },
        { "wall_loops",                 1 },
        { "top_shell_layers",           0 },
        { "bottom_shell_layers",        0 },
        { "enable_support",             0 },
        { "machine_start_gcode",        "T[initial_tool]\n" },
        { "layer_change_gcode",         "G92 E0\n" },
    });
    Print print;
    Model model;
    init_print({ TriangleMesh(its_make_cube(20., 20., 20.)) }, print, model, config);
    ModelInstance &instance = *model.objects.front()->instances.front();
    instance.set_offset(Vec3d(100., 100., instance.get_offset().z()));
    print.apply(model, config);
    std::vector<StringObjectException> warnings;
    REQUIRE(print.validate(&warnings).string.empty());
    const std::string gcode = Test::gcode(print);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.has_belt_brim());
    const std::vector<ExtrusionEntityCollection> &bands = object.belt_brim_by_layer();
    REQUIRE(bands.size() == object.layers().size());

    // The G-code Z range of each layer's extrusions, keyed by the ;Z: marker's slicing Z.
    struct ZRange { double lo = std::numeric_limits<double>::max(), hi = -std::numeric_limits<double>::max(); };
    std::vector<std::pair<double, ZRange>> layers;
    GCodeReader reader;
    reader.apply_config(print.config());
    reader.parse_buffer(gcode, [&layers](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (boost::starts_with(line.raw(), ";Z:")) {
            layers.emplace_back(std::stod(line.raw().substr(3)), ZRange{});
        } else if (! layers.empty() && line.extruding(self) && line.dist_XY(self) > 0.) {
            ZRange &r = layers.back().second;
            r.lo = std::min(r.lo, double(line.new_Z(self)));
            r.hi = std::max(r.hi, double(line.new_Z(self)));
        }
    });
    auto range_of = [&layers](coordf_t print_z) -> const ZRange * {
        for (const auto &[z, r] : layers)
            if (std::abs(z - print_z) < 1e-4 && r.lo <= r.hi)
                return &r;
        return nullptr;
    };

    const double pitch = 0.2 / std::sin(M_PI / 4.);   // a layer's advance along the belt
    size_t flat_band_layers = 0, ramped_layers = 0, measured = 0;
    for (size_t i = 0; i < object.layers().size(); ++ i) {
        const ZRange *r = range_of(object.layers()[i]->print_z);
        if (r == nullptr)
            continue;
        ++ measured;
        const double rise = r->hi - r->lo;
        INFO("layer " << i << " at " << object.layers()[i]->print_z << ": Z rises " << rise << " mm, band " << (bands[i].empty() ? "empty" : "laid"));
        if (! bands[i].empty()) {
            CHECK(rise < 1e-3);
            ++ flat_band_layers;
        } else if (rise > 0.5 * pitch)
            ++ ramped_layers;
    }
    INFO(measured << " layers measured, " << flat_band_layers << " flat with a band, " << ramped_layers << " spiralled");
    // The cube sits on the belt over its first 14 mm of slicing Z and spirals above that.
    CHECK(flat_band_layers > 20);
    CHECK(ramped_layers > 20);
}
