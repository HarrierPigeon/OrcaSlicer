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
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/Support/BeltFloorContext.hpp"
#include "libslic3r/Support/SupportDirection.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleSelector.hpp"

#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <cstddef>
#include <initializer_list>
#include <limits>
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

// The X extent of the support column (+X side, or -X with `max_x` false) at every support layer in
// a window strictly below the beam's underside and clear of the first layers (a raft-less first
// layer is padded), as (print_z, x in mm) pairs, in the object's slice frame.
std::vector<std::pair<double, double>> column_outer_edge(const PrintObject &object, bool max_x = true)
{
    std::vector<std::pair<double, double>> out;
    for (const SupportLayer *layer : object.support_layers()) {
        if (layer->support_islands.empty() || layer->print_z < 6. || layer->print_z > TOP - 2. - 1.)
            continue;
        const BoundingBox box = get_extents(layer->support_islands);
        out.emplace_back(layer->print_z, unscale<double>(max_x ? box.max.x() : box.min.x()));
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

// Support layers that carry extrusions. Every generator fills support_fills; only some fill
// support_islands.
size_t support_layers_with_fills(const PrintObject &object)
{
    size_t n = 0;
    for (const SupportLayer *layer : object.support_layers())
        if (! layer->support_fills.entities.empty())
            ++ n;
    return n;
}

// Paint every facet whose three vertices satisfy `pick` as a support enforcer on the only part.
// The part's mesh is centered on loading, so `pick` gets the mesh's own bounding box to go by.
using FacetPick = std::function<bool(const Vec3f &, const BoundingBoxf3 &)>;
void paint_enforcer(Slic3r::Model &model, const FacetPick &pick)
{
    ModelVolume &volume = *model.objects.front()->volumes.front();
    const TriangleMesh &mesh = volume.mesh();
    const BoundingBoxf3 bounds = mesh.bounding_box();
    TriangleSelector selector(mesh);
    size_t painted = 0;
    for (size_t i = 0; i < mesh.its.indices.size(); ++ i) {
        const auto &face = mesh.its.indices[i];
        if (pick(mesh.its.vertices[face[0]], bounds) && pick(mesh.its.vertices[face[1]], bounds) && pick(mesh.its.vertices[face[2]], bounds)) {
            selector.set_facet(int(i), EnforcerBlockerType::ENFORCER);
            ++ painted;
        }
    }
    REQUIRE(painted > 0);
    volume.supported_facets.set(selector);
}

// The cantilever beam's underside (2 mm below its top).
bool beam_underside(const Vec3f &v, const BoundingBoxf3 &bounds) { return std::abs(double(v.z()) - (bounds.max.z() - 2.)) < 1e-4; }

} // namespace

TEST_CASE("Support follows the layers by default", "[SupportDirection]")
{
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ cantilever() }, print, direction_config({}));
    const PrintObject &object = only_object(print);
    CHECK(print.full_print_config().opt_enum<SupportDirection>("support_direction") == sdLayers);
    REQUIRE(! object.support_layers().empty());
    CHECK(! object.support_lean().active());
    CHECK(! support_lean(object).active());
    CHECK_THAT(lean_of(column_outer_edge(object)), WithinAbs(0., 0.02));
}

TEST_CASE("A custom support direction leans the column by the configured angle", "[SupportDirection]")
{
    // Tilt Y = 30 tips the support direction 30 degrees about Y, towards +X (the build-plate tilt
    // convention). The object itself is not rotated, so the direction is the same in the slice
    // frame, and the column drifts by tan(30) mm per mm of height along +X.
    const double tilt_y = 30.;
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ cantilever() }, print, direction_config({
        { "support_direction",        "custom" },
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

TEST_CASE("Vertical support on a tilted build plate leans against the plate tilt", "[SupportDirection]")
{
    // A non-belt printer whose plate is tilted 30 degrees about Y: the layers follow the plate, and
    // "World vertical" is gravity's up, which the plate tilt options describe. The column therefore
    // leans exactly as the custom direction with the same tilt does, by tan(30) along +X.
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ cantilever() }, print, direction_config({
        { "build_plate_tilt_x",  0. },
        { "build_plate_tilt_y",  30. },
        { "support_direction",   "vertical" },
    }));
    const PrintObject &object = only_object(print);
    REQUIRE(! object.support_layers().empty());
    const SupportLean lean = support_lean(object);
    CHECK(lean.active());
    CHECK_THAT(lean.lean_deg(), WithinAbs(30., 1e-6));
    CHECK_THAT(lean_of(column_outer_edge(object)), WithinAbs(std::tan(Geometry::deg2rad(30.)), 0.05));

    // On a level plate "World vertical" is the layer normal and nothing leans.
    Slic3r::Print level;
    Slic3r::Test::init_and_process_print({ cantilever() }, level, direction_config({ { "support_direction", "vertical" } }));
    CHECK(! support_lean(only_object(level)).active());
}

TEST_CASE("The direction is fixed in the bed frame, not the object's", "[SupportDirection]")
{
    // The same tilt Y = 30 with the instance turned 180 degrees about Z: the beam now points to
    // -X, and the column still leans towards +X, so it is its -X edge that drifts by +tan(30).
    Slic3r::Print print;
    Slic3r::Model model;
    Slic3r::Test::init_print({ cantilever() }, print, model, direction_config({
        { "support_direction",        "custom" },
        { "support_direction_tilt_x", 0. },
        { "support_direction_tilt_y", 30. },
    }));
    model.objects.front()->instances.front()->set_rotation(Z, M_PI);
    print.apply(model, print.full_print_config());
    print.process();
    const PrintObject &object = only_object(print);
    REQUIRE(! object.support_layers().empty());
    CHECK_THAT(lean_of(column_outer_edge(object, false)), WithinAbs(std::tan(Geometry::deg2rad(30.)), 0.05));
}

TEST_CASE("A direction along the layer normal changes nothing", "[SupportDirection]")
{
    // On a flat bed the bed normal is the layer normal, so "vertical" and a custom 0/0 both leave
    // the support exactly where the default puts it.
    const auto direction = GENERATE(as<std::string>{}, "vertical", "custom");
    INFO(direction);
    Slic3r::Print plain, same;
    Slic3r::Test::init_and_process_print({ cantilever() }, plain, direction_config({}));
    Slic3r::Test::init_and_process_print({ cantilever() }, same,  direction_config({
        { "support_direction",        direction },
        { "support_direction_tilt_x", 0. },
        { "support_direction_tilt_y", 0. },
    }));
    const PrintObject &a = only_object(plain);
    const PrintObject &b = only_object(same);
    CHECK(! support_lean(b).active());
    REQUIRE(a.support_layer_count() == b.support_layer_count());
    for (size_t i = 0; i < a.support_layer_count(); ++ i) {
        INFO("support layer " << i);
        CHECK(a.support_layers()[i]->support_islands == b.support_layers()[i]->support_islands);
    }
}

TEST_CASE("The support lean is limited by the configured maximum", "[SupportDirection]")
{
    // Tilt Y = 80 asks for an 80 degree lean towards +X; the column gets the maximum instead, 45 by
    // default, and keeps its heading.
    const double max_lean = GENERATE(45., 60.);
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ cantilever() }, print, direction_config({
        { "support_direction",          "custom" },
        { "support_direction_tilt_x",   0. },
        { "support_direction_tilt_y",   80. },
        { "support_direction_max_lean", max_lean },
    }));
    const PrintObject &object = only_object(print);
    REQUIRE(! object.support_layers().empty());
    const SupportLean lean = support_lean(object);
    INFO("max lean " << max_lean);
    CHECK(lean.clamped);
    CHECK_THAT(lean.lean_deg(), WithinAbs(max_lean, 1e-6));
    CHECK(lean.slope.x() > 0.);
    CHECK_THAT(lean.slope.y(), WithinAbs(0., 1e-9));
    CHECK_THAT(lean_of(column_outer_edge(object)), WithinAbs(std::tan(Geometry::deg2rad(max_lean)), 0.05));
}

TEST_CASE("Slicing warns about a limited or steep lean", "[SupportDirection]")
{
    const auto [tilt_y, max_lean, warnings] = GENERATE(table<double, double, size_t>({
        { 30., 45., 0 },   // within the limit and not steep
        { 80., 45., 1 },   // pulled back to the limit
        { 80., 60., 1 },   // pulled back to the limit, which is also steep: one warning
        { 50., 60., 1 },   // admitted, but past the 45 degrees a column can overhang itself
    }));
    INFO("tilt " << tilt_y << " max lean " << max_lean);
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ cantilever() }, print, direction_config({
        { "support_direction",          "custom" },
        { "support_direction_tilt_x",   0. },
        { "support_direction_tilt_y",   tilt_y },
        { "support_direction_max_lean", max_lean },
    }));
    CHECK(only_object(print).step_state_with_warnings(posSupportMaterial).warnings.size() == warnings);
}

TEST_CASE("Every support generator leaves the object layers exactly as sliced after a leaning run", "[SupportDirection][belt]")
{
    // The lean is produced by shearing the object layers around the generator and shearing them
    // back. Whatever the generator, the layers a leaning run leaves behind must be the ones a plain
    // run has -- otherwise the perimeters and infill already generated would print displaced. On a
    // belt the normal generator lifts the layers' print_z for its run, which the un-shear must not
    // be fooled by.
    const auto [type, style, belt] = GENERATE(table<const char*, const char*, bool>({
        { "normal(auto)", "snug",      false },
        { "tree(auto)",   "tree_slim", false },
        { "tree(auto)",   "organic",   false },
        { "normal(auto)", "snug",      true },
        { "tree(auto)",   "tree_slim", true },
        { "tree(auto)",   "organic",   true },
    }));
    INFO(type << " / " << style << (belt ? " on a belt" : ""));
    const std::initializer_list<ConfigBase::SetDeserializeItem> generator = {
        { "support_type",  type },
        { "support_style", style },
    };
    const DynamicPrintConfig plain_config = belt ? belt_direction_config(generator) : direction_config(generator);
    DynamicPrintConfig       leaning_config = plain_config;
    if (belt)
        leaning_config.set_deserialize_strict({ { "support_direction", "vertical" } });
    else
        leaning_config.set_deserialize_strict({ { "support_direction", "custom" }, { "support_direction_tilt_x", 0. }, { "support_direction_tilt_y", 30. } });
    Slic3r::Print plain, leaning;
    Slic3r::Test::init_and_process_print({ cantilever() }, plain,   plain_config);
    Slic3r::Test::init_and_process_print({ cantilever() }, leaning, leaning_config);
    const PrintObject &a = only_object(plain);
    const PrintObject &b = only_object(leaning);
    INFO("belt z offset " << b.belt_global_z_offset());
    REQUIRE(support_lean(b).active());
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
    const auto [type, style] = GENERATE(table<const char*, const char*>({
        { "normal(auto)", "snug" },
        { "tree(auto)",   "tree_slim" },
        { "tree(auto)",   "organic" },
    }));
    INFO(type << " / " << style);
    Slic3r::Print print;
    Slic3r::Test::init_and_process_print({ cantilever() }, print, belt_direction_config({
        { "support_type",      type },
        { "support_style",     style },
        { "support_direction", "vertical" },
    }));
    const PrintObject &object = only_object(print);
    REQUIRE(! object.support_layers().empty());
    CHECK(support_layers_with_fills(object) > 0);
    CHECK(! object.support_lean().active());   // scoped to generation

    const double      shear = object.slicing_parameters().belt_floor_shear_factor;
    REQUIRE(shear > 0.5);                      // the belt tilt is in effect, and the floor is at y >= cutoff
    const SupportLean lean  = support_lean(object);
    CHECK(lean.active());
    CHECK(! lean.clamped);
    CHECK_THAT(lean.lean_deg(), WithinAbs(45., 0.5));
    CHECK_THAT(lean.slope.x(), WithinAbs(0., 1e-6));
    CHECK_THAT(lean.slope.y(), WithinAbs(- shear, 1e-6));

    // Everything is back in the slice frame now, so the belt floor in that frame bounds the
    // support: no extrusion may lie on the belt side of the floor at its own print_z beyond the
    // layer that rests on it (one layer pitch along the floor, plus clipping slack), and some
    // layer has to reach the floor, else the column hangs in the air.
    BeltFloorContext floor;
    REQUIRE(floor.init(object.slicing_parameters(), print.config()));
    double worst_intrusion = 0., closest = std::numeric_limits<double>::max();
    for (const SupportLayer *layer : object.support_layers()) {
        const Polylines paths = layer->support_fills.as_polylines();
        if (paths.empty())
            continue;
        const double cutoff = floor.cutoff_u(layer->print_z);
        const double edge   = unscale<double>(get_extents(paths).max.y());
        worst_intrusion = std::max(worst_intrusion, edge - cutoff);
        closest         = std::min(closest, cutoff - edge);
    }
    INFO("deepest support reach past the belt floor " << worst_intrusion << " mm, closest approach " << closest << " mm");
    CHECK(worst_intrusion < 1.);
    CHECK(closest < 1.);
}

TEST_CASE("A bed-vertical wall on a belt is not an overhang for vertical support", "[SupportDirection][belt]")
{
    // A plain post: sliced at 45 degrees its bed-vertical walls are 45 degree overhangs relative
    // to the layer normal, which a 50 degree threshold flags. Judged against the belt normal they
    // are walls and need nothing, so vertical support generates no extrusions at all. Whether
    // layer-following support treats them as overhangs is each generator's own belt compensation,
    // pinned here as it stands: the tree generators compensate, the normal generator does not.
    const auto [type, style, supported_when_following_layers] = GENERATE(table<const char*, const char*, bool>({
        { "normal(auto)", "snug",      true },
        { "tree(auto)",   "tree_slim", false },
        { "tree(auto)",   "organic",   false },
    }));
    INFO(type << " / " << style);
    Slic3r::Print vertical, layers;
    Slic3r::Test::init_and_process_print({ make_cube(10., 10., 30.) }, vertical, belt_direction_config({
        { "support_type",            type },
        { "support_style",           style },
        { "support_threshold_angle", 50 },
        { "support_direction",       "vertical" },
    }));
    REQUIRE(! only_object(vertical).layers().empty());
    CHECK(support_layers_with_fills(only_object(vertical)) == 0);

    Slic3r::Test::init_and_process_print({ make_cube(10., 10., 30.) }, layers, belt_direction_config({
        { "support_type",            type },
        { "support_style",           style },
        { "support_threshold_angle", 50 },
        { "support_direction",       "layers" },
    }));
    CHECK((support_layers_with_fills(only_object(layers)) > 0) == supported_when_following_layers);
}

TEST_CASE("Painted enforcers follow the layers into the sheared frame", "[SupportDirection]")
{
    // While the support leans, the painted areas a generator asks for must sit where the sheared
    // layers are: each layer's projection moved by the shift that layer received, once.
    Slic3r::Print print;
    Slic3r::Model model;
    const DynamicPrintConfig config = direction_config({
        { "support_direction",        "custom" },
        { "support_direction_tilt_x", 0. },
        { "support_direction_tilt_y", 30. },
    });
    Slic3r::Test::init_print({ cantilever() }, print, model, config);
    paint_enforcer(model, beam_underside);
    print.apply(model, config);
    print.process();

    PrintObject &object = *print.get_object(0);
    std::vector<Polygons> plain;
    object.project_and_append_custom_facets(false, EnforcerBlockerType::ENFORCER, plain);
    REQUIRE(std::any_of(plain.begin(), plain.end(), [](const Polygons &p) { return ! p.empty(); }));
    {
        // The scope moves the (already generated) support layers back with the object layers when
        // it ends; the print is discarded afterwards, so that does no harm here.
        ScopedSupportShear shear(object);
        REQUIRE(shear.active());
        std::vector<Polygons> sheared;
        object.project_and_append_custom_facets(false, EnforcerBlockerType::ENFORCER, sheared);
        REQUIRE(sheared.size() == plain.size());
        for (size_t i = 0; i < plain.size(); ++ i) {
            Polygons expected = plain[i];
            translate(expected, object.support_lean_shift(i));
            INFO("layer " << i);
            CHECK(sheared[i] == expected);
        }
    }
}

TEST_CASE("Painted enforcers on a belt stay with the layers the normal generator lifts", "[SupportDirection][belt]")
{
    // The normal generator runs with the belt's global z offset lifted off the object layers,
    // while the shear was applied at the global print_z. With manual support, the painted beam
    // underside is the only thing that produces support; placed off the belt's z origin, the
    // enforcer only meets the beam if it is sheared by what the layers were, not by what their
    // print_z says during the run.
    Slic3r::Print print;
    Slic3r::Model model;
    const DynamicPrintConfig config = belt_direction_config({
        { "support_type",      "normal(manual)" },
        { "support_direction", "vertical" },
    });
    Slic3r::Test::init_print({ cantilever() }, print, model, config);
    paint_enforcer(model, beam_underside);
    ModelInstance &instance = *model.objects.front()->instances.front();
    instance.set_offset(instance.get_offset() + Vec3d(0., 80., 0.));
    print.apply(model, config);
    print.process();

    const PrintObject &object = only_object(print);
    INFO("belt z offset " << object.belt_global_z_offset());
    REQUIRE(std::abs(object.belt_global_z_offset()) > 20.);
    REQUIRE(support_lean(object).active());
    CHECK(support_layers_with_fills(object) > 0);
}

TEST_CASE("Painted vertical faces lean with the column in classic tree support", "[SupportDirection]")
{
    // Classic tree support reads painted vertical faces as points, not areas. A post with its +X
    // wall painted and the support leaning towards +X: every tip and branch must stay on the -X
    // side of that wall, which a column leaning up to it does, and a point left un-sheared does
    // not (it would come out up to z * tan(30) past the wall).
    Slic3r::Print print;
    Slic3r::Model model;
    const DynamicPrintConfig config = direction_config({
        { "support_type",             "tree(manual)" },
        { "support_style",            "tree_slim" },
        { "support_direction",        "custom" },
        { "support_direction_tilt_x", 0. },
        { "support_direction_tilt_y", 30. },
    });
    Slic3r::Test::init_print({ make_cube(10., 10., 30.) }, print, model, config);
    paint_enforcer(model, [](const Vec3f &v, const BoundingBoxf3 &bounds) { return std::abs(double(v.x()) - bounds.max.x()) < 1e-4; });
    print.apply(model, config);
    print.process();

    const PrintObject &object = only_object(print);
    REQUIRE(support_layers_with_fills(object) > 0);
    // The wall's x in the slice frame: the object is centered, so read it off the layers.
    double wall_x = -std::numeric_limits<double>::max();
    for (const Layer *layer : object.layers())
        wall_x = std::max(wall_x, unscale<double>(get_extents(layer->lslices).max.x()));
    double support_max_x = -std::numeric_limits<double>::max();
    for (const SupportLayer *layer : object.support_layers()) {
        const Polylines paths = layer->support_fills.as_polylines();
        if (! paths.empty())
            support_max_x = std::max(support_max_x, unscale<double>(get_extents(paths).max.x()));
    }
    INFO("wall at x " << wall_x << ", support reaches x " << support_max_x);
    CHECK(support_max_x < wall_x + 4.);
}
