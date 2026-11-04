#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/Support/BeltFloorContext.hpp"
#include "libslic3r/Support/SupportDirection.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleSelector.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

// Support direction: support columns leaning along a direction other than the layer normal.
//
// The fixture is a cantilever: a pillar on the bed with a beam sticking out of its top. Everything
// under the beam past the pillar needs support, and that support forms one column whose OUTER edge
// (the beam's free end) is trimmed by nothing, so where that edge sits at each height says exactly
// which way the column leans. The tests read the column off the support layers directly; G-code is
// only generated to prove the leaning support still emits.

namespace {

constexpr double PILLAR = 8.;    // pillar footprint and the beam's width
constexpr double BEAM   = 40.;   // beam length along +X
constexpr double TOP    = 20.;   // beam top; the beam is 2mm thick, so its underside is at 18

TriangleMesh cantilever()
{
    TriangleMesh pillar = make_cube(PILLAR, PILLAR, TOP);
    TriangleMesh beam   = make_cube(BEAM, PILLAR, 2.);
    beam.translate(0., 0., float(TOP - 2.));
    pillar.merge(beam);
    return pillar;
}

DynamicPrintConfig direction_config(std::initializer_list<ConfigBase::SetDeserializeItem> extra)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_support",          "1" },
        { "support_type",            "normal(auto)" },
        { "support_style",           "snug" },
        { "support_threshold_angle", "30" },
        { "layer_height",            "0.2" },
        { "skirt_loops",             "0" },
    });
    config.set_deserialize_strict(extra);
    return config;
}

// The belt keys the belt cases in test_print.cpp use: a belt tilted 45 degrees about X.
DynamicPrintConfig belt_direction_config(std::initializer_list<ConfigBase::SetDeserializeItem> extra)
{
    DynamicPrintConfig config = direction_config({
        { "belt_printer",               1 },
        { "belt_slice_rotation",        "x" },
        { "belt_slice_rotation_angle",  45 },
        { "gcode_remap_x",              "rev_x" },
        { "gcode_remap_y",              "pos_z" },
        { "gcode_remap_z",              "pos_y" },
        { "initial_layer_print_height", 0.2 },
        { "z_hop",                      0 },
        { "machine_start_gcode",        "T[initial_tool]\n" },
        { "layer_change_gcode",         "G92 E0\n" },
    });
    config.set_deserialize_strict(extra);
    return config;
}

// The +X extent of the support column at every support layer in a window that is strictly below
// the beam's underside and clear of the first layers (a raft-less first layer is padded), as
// (print_z, max x in mm) pairs, in the object's slice frame.
std::vector<std::pair<double, double>> column_outer_edge(const PrintObject &object)
{
    std::vector<std::pair<double, double>> out;
    for (const SupportLayer *layer : object.support_layers()) {
        if (layer->support_islands.empty() || layer->print_z < 6. || layer->print_z > TOP - 2. - 1.)
            continue;
        out.emplace_back(layer->print_z, unscale<double>(get_extents(layer->support_islands).max.x()));
    }
    return out;
}

// Least-squares slope of x against z: the column's lean, in mm of drift per mm of height.
double lean_of(const std::vector<std::pair<double, double>> &edge)
{
    REQUIRE(edge.size() >= 10);
    double sz = 0., sx = 0., szz = 0., szx = 0.;
    for (const auto &[z, x] : edge) {
        sz += z; sx += x; szz += z * z; szx += z * x;
    }
    const double n = double(edge.size());
    return (n * szx - sz * sx) / (n * szz - sz * sz);
}

const PrintObject &only_object(const Slic3r::Print &print)
{
    REQUIRE(print.objects().size() == 1);
    return *print.objects().front();
}

size_t support_layers_with_islands(const PrintObject &object)
{
    size_t n = 0;
    for (const SupportLayer *layer : object.support_layers())
        if (! layer->support_islands.empty())
            ++ n;
    return n;
}

} // namespace

TEST_CASE("Support follows the layers by default", "[SupportDirection]")
{
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ cantilever() }, print, direction_config({}));
    const PrintObject &object = only_object(print);
    REQUIRE(! object.support_layers().empty());
    CHECK(! object.support_lean().active());
    CHECK(! support_lean(object).active());
    CHECK_THAT(lean_of(column_outer_edge(object)), WithinAbs(0., 0.02));
}

TEST_CASE("A custom support direction leans the column by the configured angle", "[SupportDirection]")
{
    // Tilt Y = 30 tips the support axis 30 degrees about Y, towards +X (the build-plate tilt
    // convention). The object itself is not rotated, so the axis is the same in the slice frame,
    // and the column drifts by tan(30) mm per mm of height along +X.
    const double tilt_y = 30.;
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ cantilever() }, print, direction_config({
        { "support_direction",   "custom" },
        { "support_direction_tilt_x", 0. },
        { "support_direction_tilt_y", tilt_y },
    }));
    const PrintObject &object = only_object(print);
    REQUIRE(! object.support_layers().empty());
    CHECK(! object.support_lean().active());   // the shear is scoped to generation and is gone now

    const Vec3d  axis     = Geometry::tilt_up_direction(0., tilt_y);
    const double expected = axis.x() / axis.z();
    CHECK_THAT(expected, WithinAbs(std::tan(Geometry::deg2rad(30.)), 1e-6));
    CHECK_THAT(lean_of(column_outer_edge(object)), WithinAbs(expected, 0.05));

    SECTION("and the leaning support is emitted as support G-code") {
        const std::string g = Slic3r::Test::gcode(print);
        CHECK(! Slic3r::Test::layers_with_role(g, "support").empty());
    }
}

TEST_CASE("A custom direction equal to the layer normal changes nothing", "[SupportDirection]")
{
    Slic3r::Print plain, custom;
    Slic3r::Test::init_and_process_print({ cantilever() }, plain,  direction_config({}));
    Slic3r::Test::init_and_process_print({ cantilever() }, custom, direction_config({
        { "support_direction",   "custom" },
        { "support_direction_tilt_x", 0. },
        { "support_direction_tilt_y", 0. },
    }));
    const auto edge_plain  = column_outer_edge(only_object(plain));
    const auto edge_custom = column_outer_edge(only_object(custom));
    REQUIRE(edge_plain.size() == edge_custom.size());
    for (size_t i = 0; i < edge_plain.size(); ++ i) {
        INFO("layer " << i << " at z " << edge_plain[i].first);
        CHECK_THAT(edge_custom[i].second, WithinAbs(edge_plain[i].second, 1e-6));
    }
}

TEST_CASE("The support lean is limited by the configured maximum", "[SupportDirection]")
{
    // Tilt Y = 80 asks for an 80 degree lean; the column gets the maximum instead, 45 by default.
    const double max_lean = GENERATE(45., 60.);
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ cantilever() }, print, direction_config({
        { "support_direction",          "custom" },
        { "support_direction_tilt_x",        0. },
        { "support_direction_tilt_y",        80. },
        { "support_direction_max_lean", max_lean },
    }));
    const PrintObject &object = only_object(print);
    REQUIRE(! object.support_layers().empty());
    const SupportLean lean = support_lean(object);
    INFO("max lean " << max_lean);
    CHECK(lean.clamped);
    CHECK_THAT(lean.lean_deg(), WithinAbs(max_lean, 1e-6));
    CHECK_THAT(std::abs(lean_of(column_outer_edge(object))), WithinAbs(std::tan(Geometry::deg2rad(max_lean)), 0.05));
}

TEST_CASE("Every support generator leaves the object layers exactly as sliced after a leaning run", "[SupportDirection]")
{
    // The lean is produced by shearing the object layers around the generator and shearing them
    // back. Whatever the generator, the layers a leaning run leaves behind must be the ones a plain
    // run has -- otherwise the perimeters and infill already generated would print displaced.
    const auto [type, style] = GENERATE(table<const char*, const char*>({
        { "normal(auto)", "snug" },
        { "tree(auto)",   "tree_slim" },
        { "tree(auto)",   "organic" },
    }));
    INFO(type << " / " << style);
    Slic3r::Print plain, leaning;
    Slic3r::Test::init_and_process_print({ cantilever() }, plain, direction_config({
        { "support_type",  type },
        { "support_style", style },
    }));
    Slic3r::Test::init_and_process_print({ cantilever() }, leaning, direction_config({
        { "support_type",        type },
        { "support_style",       style },
        { "support_direction",   "custom" },
        { "support_direction_tilt_x", 0. },
        { "support_direction_tilt_y", 30. },
    }));
    const PrintObject &a = only_object(plain);
    const PrintObject &b = only_object(leaning);
    REQUIRE(! b.support_layers().empty());
    REQUIRE(a.layer_count() == b.layer_count());
    for (size_t i = 0; i < a.layer_count(); ++ i) {
        const Layer *la = a.get_layer(int(i));
        const Layer *lb = b.get_layer(int(i));
        INFO("layer " << i << " at z " << la->print_z);
        REQUIRE(la->lslices.size() == lb->lslices.size());
        for (size_t k = 0; k < la->lslices.size(); ++ k)
            CHECK(la->lslices[k] == lb->lslices[k]);
        REQUIRE(la->region_count() == lb->region_count());
        for (size_t r = 0; r < la->region_count(); ++ r) {
            const ExtrusionEntityCollection &pa = la->get_region(int(r))->perimeters;
            const ExtrusionEntityCollection &pb = lb->get_region(int(r))->perimeters;
            REQUIRE(pa.entities.size() == pb.entities.size());
            for (size_t e = 0; e < pa.entities.size(); ++ e)
                CHECK(pa.entities[e]->first_point() == pb.entities[e]->first_point());
        }
    }
}

TEST_CASE("Vertical support on a belt grows perpendicular to the belt and lands on it", "[SupportDirection][belt]")
{
    // The belt is tilted 45 degrees about X, so the object is rotated by that angle before slicing
    // and the bed normal is the belt normal in the slice frame. BeltFloorContext places the belt
    // at z = shear * y + const (floor_print_z), whose upward normal is (0, -shear, 1): the lean
    // that makes a column perpendicular to the belt is therefore (0, -shear) mm per mm of height,
    // 45 degrees from the layer normal -- exactly the default 45 degree limit, which admits it.
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ cantilever() }, print, belt_direction_config({
        { "support_direction", "vertical" },
    }));
    const PrintObject &object = only_object(print);
    REQUIRE(! object.support_layers().empty());
    CHECK(support_layers_with_islands(object) > 0);
    CHECK(! object.support_lean().active());   // scoped to generation

    const double      shear = object.slicing_parameters().belt_floor_shear_factor;
    REQUIRE(std::abs(shear) > 0.5);            // the belt tilt is actually in effect
    const SupportLean lean  = support_lean(object);
    CHECK(lean.active());
    CHECK(! lean.clamped);
    CHECK_THAT(lean.lean_deg(), WithinAbs(45., 0.5));
    CHECK_THAT(lean.slope.x(), WithinAbs(0., 1e-6));
    CHECK_THAT(lean.slope.y(), WithinAbs(- shear, 1e-6));

    // Everything is back in the slice frame now, so the belt floor in that frame bounds the
    // support: no island may lie on the belt side of the floor at its own print_z, beyond the
    // layer that rests on it (one layer pitch along the floor, plus clipping slack).
    BeltFloorContext floor;
    REQUIRE(floor.init(object.slicing_parameters(), print.config()));
    double worst_intrusion = 0.;
    for (const SupportLayer *layer : object.support_layers()) {
        if (layer->support_islands.empty())
            continue;
        const ExPolygons below = intersection_ex(layer->support_islands, floor.surface_polygon(layer->print_z));
        if (below.empty())
            continue;
        const double cutoff = floor.cutoff_u(layer->print_z);
        const double edge   = unscale<double>(floor.from_axis() == 0 ? get_extents(below).max.x() : get_extents(below).max.y());
        worst_intrusion = std::max(worst_intrusion, edge - cutoff);
    }
    INFO("deepest support reach past the belt floor " << worst_intrusion << " mm");
    CHECK(worst_intrusion < 1.);
}

TEST_CASE("A bed-vertical wall on a belt is not an overhang for vertical support", "[SupportDirection][belt]")
{
    // A plain post: sliced at 45 degrees its bed-vertical walls are 45 degree overhangs relative
    // to the layer normal, but perpendicular to the belt normal. Judged against the support
    // direction they need nothing, so vertical support generates no islands at all.
    const auto [type, style] = GENERATE(table<const char*, const char*>({
        { "normal(auto)", "snug" },
        { "tree(auto)",   "tree_slim" },
        { "tree(auto)",   "organic" },
    }));
    INFO(type << " / " << style);
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ make_cube(10., 10., 30.) }, print, belt_direction_config({
        { "support_type",      type },
        { "support_style",     style },
        { "support_direction", "vertical" },
    }));
    const PrintObject &object = only_object(print);
    REQUIRE(! object.layers().empty());
    CHECK(support_layers_with_islands(object) == 0);

    // Whether layer-following support treats that wall as an overhang is recorded, not required:
    // it documents today's behaviour of the generators' own belt tilt compensation.
    Slic3r::Print layers;
    Slic3r::Test::init_and_process_print({ make_cube(10., 10., 30.) }, layers, belt_direction_config({
        { "support_type",      type },
        { "support_style",     style },
        { "support_direction", "layers" },
    }));
    const size_t n = support_layers_with_islands(only_object(layers));
    INFO("layer-following support puts islands on " << n << " support layers of a bed-vertical post on a belt");
    CHECK_NOFAIL(n == 0);
}

TEST_CASE("A leaning run shifts an enforcer volume once", "[SupportDirection]")
{
    // The generators hand project_and_append_custom_facets() the enforcer volumes that
    // slice_support_volumes() has already shifted into the sheared frame. With nothing painted it
    // must leave them alone; shifting them again put the enforcer z * slope off at every layer.
    Slic3r::Print print;
    Slic3r::Model model;
    const DynamicPrintConfig config = direction_config({
        { "support_direction",   "custom" },
        { "support_direction_tilt_x", 0. },
        { "support_direction_tilt_y", 30. },
    });
    Slic3r::Test::init_print({ cantilever() }, print, model, config);
    TriangleMesh block = make_cube(4., 4., 4.);
    block.translate(20.f, 2.f, 8.f);
    model.objects.front()->add_volume(std::move(block), ModelVolumeType::SUPPORT_ENFORCER);
    print.apply(model, print.full_print_config());
    print.process();

    PrintObject &object = *print.get_object(0);
    ScopedSupportShear shear(object);
    REQUIRE(shear.active());
    std::vector<Polygons> enforcers = object.slice_support_enforcers();
    REQUIRE(std::any_of(enforcers.begin(), enforcers.end(), [](const Polygons &p) { return ! p.empty(); }));
    const std::vector<Polygons> sliced = enforcers;
    object.project_and_append_custom_facets(false, EnforcerBlockerType::ENFORCER, enforcers);
    CHECK(enforcers == sliced);
}
