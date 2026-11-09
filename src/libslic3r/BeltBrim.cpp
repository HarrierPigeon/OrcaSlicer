#include <limits>
#include "BeltBrim.hpp"

#include "ClipperUtils.hpp"
#include "Flow.hpp"
#include "Layer.hpp"
#include "Polygon.hpp"
#include "Print.hpp"
#include "ShortestPath.hpp"
#include "Surface.hpp"
#include "SurfaceCollection.hpp"
#include "Support/BeltFloorContext.hpp"
#include "BoundingBox.hpp"
#include "ExPolygon.hpp"
#include "ExtrusionEntity.hpp"
#include "ExtrusionEntityCollection.hpp"
#include "Point.hpp"
#include "Polyline.hpp"
#include "PrintConfig.hpp"
#include "libslic3r.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <utility>
#include <vector>

namespace Slic3r {

// ---------------------------------------------------------------- scaling

static inline Point scale_u_point(const Point &p, int from_axis, double factor)
{
    // llround, not a cast: casting truncates toward zero, so a round trip would
    // walk every vertex toward the origin by up to one unit per pass.
    return from_axis == 0 ?
        Point(coord_t(std::llround(double(p.x()) * factor)), p.y()) :
        Point(p.x(), coord_t(std::llround(double(p.y()) * factor)));
}

static inline void scale_u_polygon(Polygon &poly, int from_axis, double factor)
{
    for (Point &p : poly.points)
        p = scale_u_point(p, from_axis, factor);
}

ExPolygons belt_scale_u(const ExPolygons &src, const BeltBrimFrame &frame, double factor)
{
    ExPolygons out = src;
    for (ExPolygon &ex : out) {
        scale_u_polygon(ex.contour, frame.from_axis, factor);
        for (Polygon &hole : ex.holes)
            scale_u_polygon(hole, frame.from_axis, factor);
    }
    return out;
}

Polylines belt_scale_u(const Polylines &src, const BeltBrimFrame &frame, double factor)
{
    Polylines out = src;
    for (Polyline &pl : out)
        for (Point &p : pl.points)
            p = scale_u_point(p, frame.from_axis, factor);
    return out;
}

// ---------------------------------------------------------------- sweep

ExPolygons sweep_ex(const ExPolygons &src, const Point &t)
{
    if (src.empty())
        return {};
    if (t == Point(0, 0))
        return src;

    // One parallelogram per boundary edge.  Together with P and P + t these
    // cover the Minkowski sum exactly: for any q = p + s*t with p in P and
    // s in [0, 1], let s* be the smallest lambda >= 0 with q - lambda*t in P.
    // Either s* == 0 (so q is in P) or q - s* * t lies on some boundary edge e,
    // putting q in that edge's parallelogram.  Hole edges must be included, or
    // holes narrower than t along t would wrongly survive the sweep.
    Polygons quads;
    for (const ExPolygon &ex : src)
        for (size_t c = 0; c < ex.num_contours(); ++ c)
            for (const Line &e : ex.contour_or_hole(c).lines()) {
                if (e.a == e.b)
                    continue;
                Polygon q;
                q.points = { e.a, e.b, e.b + t, e.a + t };
                // The non-zero fill rule counts a clockwise ring as -1, which
                // would punch a hole instead of adding material.  Edges parallel
                // to t give a zero-area quad; Clipper discards those harmlessly.
                if (q.is_clockwise())
                    q.reverse();
                quads.emplace_back(std::move(q));
            }

    ExPolygons shifted = src;
    for (ExPolygon &ex : shifted)
        ex.translate(t);

    // union_ex(ExPolygons, Polygons) uses pftNonZero, which is the fill rule the
    // argument above relies on.
    return union_ex(union_ex(src, shifted), quads);
}

// ---------------------------------------------------------------- brim region

ExPolygons belt_brim_region(const ExPolygons    &footprint_flat,
                            bool                 has_outer,
                            bool                 has_inner,
                            coord_t              brim_width,
                            coord_t              object_gap,
                            coord_t              leading,
                            coord_t              lateral,
                            const BeltBrimFrame &frame)
{
    if (footprint_flat.empty() || (! has_outer && ! has_inner))
        return {};

    ExPolygons out;

    if (has_outer) {
        // Offset the outer ring from the contours only, so a hole cannot punch
        // through it.  Same reasoning as the plate brim in Brim.cpp.
        Polygons contours;
        contours.reserve(footprint_flat.size());
        for (const ExPolygon &ex : footprint_flat)
            contours.emplace_back(ex.contour);

        // Inner and outer boundary offset from the same polygon, to avoid
        // round-off mismatch between them.
        ExPolygons inner = offset_ex(contours, float(object_gap), jtRound, SCALED_RESOLUTION);

        // Close the interior before offsetting outwards.  A belt contact patch is often a
        // narrow, broken-up strip, and the offset rings of two islands less than
        // 2 x brim_width apart merge and fill the space between them - space that lies
        // UNDER the part, which is not what "outer brim" means.  Closing also swallows
        // holes in the patch for the same reason.  Concavity-filling only, so an apron or
        // any other outward protrusion is untouched.
        ExPolygons envelope = brim_width > 0 ? closing_ex(inner, float(brim_width)) : inner;

        ExPolygons base  = envelope;
        if (leading > 0) {
            // Sweep downhill from the gapped keep-out, so the apron is contiguous with
            // the ring instead of starting inside the gap.
            const Point t = frame.from_axis == 0 ?
                Point(frame.downhill_sign() * leading, 0) :
                Point(0, frame.downhill_sign() * leading);
            base = union_ex(base, sweep_ex(envelope, t));
        }
        if (lateral > 0) {
            // Across the belt, both ways.  Swept from `base` so the apron is widened
            // too, and in the flattened frame the cross-belt axis is unscaled, so this
            // distance is already a true on-belt distance.
            const Point t = frame.from_axis == 0 ? Point(0, lateral) : Point(lateral, 0);
            ExPolygons widened = union_ex(sweep_ex(base, t), sweep_ex(base, Point(-t.x(), -t.y())));
            base = union_ex(base, to_polygons(widened));
        }
        ExPolygons outer = offset_ex(base, float(brim_width), jtRound, SCALED_RESOLUTION);
        expolygons_append(out, diff_ex(outer, envelope));
    }

    if (has_inner) {
        // Holes reversed so a negative offset grows inward, mirroring Brim.cpp.
        // No apron here: an apron growing into a hole interior is never useful.
        Polygons holes;
        for (const ExPolygon &ex : footprint_flat)
            polygons_append(holes, ex.holes);
        polygons_reverse(holes);
        if (! holes.empty()) {
            ExPolygons hole_inner = offset_ex(holes, - float(brim_width + object_gap));
            ExPolygons hole_outer = offset_ex(holes, - float(object_gap));
            expolygons_append(out, intersection_ex(diff_ex(hole_outer, hole_inner), holes));
        }
    }

    return union_ex(out);
}

// ---------------------------------------------------------------- line lattice

std::vector<coord_t> belt_brim_line_positions(coord_t u_lo,
                                              coord_t u_hi,
                                              coord_t pitch_u,
                                              coord_t u_anchor)
{
    std::vector<coord_t> out;
    if (pitch_u <= 0 || u_hi <= u_lo)
        return out;

    // Walk the lattice from just below u_lo.  Integer arithmetic throughout, so
    // the half-open interval needs no epsilon: a point landing exactly on u_hi
    // belongs to the next band.
    int64_t k = int64_t(std::floor(double(u_lo - u_anchor) / double(pitch_u))) - 1;
    while (u_anchor + coord_t(k) * pitch_u < u_lo)
        ++ k;
    for (;; ++ k) {
        const coord_t u = u_anchor + coord_t(k) * pitch_u;
        if (u >= u_hi)
            break;
        out.emplace_back(u);
    }
    return out;
}

// ---------------------------------------------------------------- pipeline

// A band of the belt surface as an explicit box, clamped to `bounds` along the
// shear axis.  Deliberately not BeltFloorContext::surface_polygon(): those
// half-planes span +-1000 mm, which is wasteful to clip against and dangerous to
// feed through the flattening scale.
static Polygon band_box(const BoundingBox &bounds, int from_axis, coordf_t u_lo, coordf_t u_hi)
{
    coord_t lo = scale_(u_lo);
    coord_t hi = scale_(u_hi);
    const coord_t bmin = from_axis == 0 ? bounds.min.x() : bounds.min.y();
    const coord_t bmax = from_axis == 0 ? bounds.max.x() : bounds.max.y();
    lo = std::max(lo, bmin);
    hi = std::min(hi, bmax);
    Polygon poly;
    if (hi <= lo)
        return poly;
    if (from_axis == 0)
        poly.points = { Point(lo, bounds.min.y()), Point(hi, bounds.min.y()),
                        Point(hi, bounds.max.y()), Point(lo, bounds.max.y()) };
    else
        poly.points = { Point(bounds.min.x(), lo), Point(bounds.max.x(), lo),
                        Point(bounds.max.x(), hi), Point(bounds.min.x(), hi) };
    return poly;
}

ExPolygons belt_brim_clip_leading_edge(const ExPolygons &region, const BeltBrimFrame &frame, coordf_t u_cut)
{
    if (region.empty())
        return region;
    BoundingBox keep_bb = get_extents(region);
    keep_bb.offset(scale_(1.));
    const bool    low_side = frame.downhill_sign() < 0;   // downhill is -u
    const Polygon keep = band_box(keep_bb, frame.from_axis,
        low_side ? unscale<double>(frame.from_axis == 0 ? keep_bb.min.x() : keep_bb.min.y()) : u_cut,
        low_side ? u_cut : unscale<double>(frame.from_axis == 0 ? keep_bb.max.x() : keep_bb.max.y()));
    return keep.empty() ? ExPolygons{} : intersection_ex(region, Polygons{ keep });
}

// One region the per-band line generator lays lines in, and the role those lines take.
struct BeltBrimRegion
{
    ExPolygons    region;   // object-local slicing XY
    Polygons      polys;    // to_polygons(region), once
    BoundingBox   bbox;
    ExtrusionRole role = erBrim;

    bool empty() const { return region.empty(); }
    void set(ExPolygons &&r, ExtrusionRole role_)
    {
        region = std::move(r);
        polys  = to_polygons(region);
        bbox   = get_extents(region);
        role   = role_;
    }
};

// Everything the per-band line generator needs, gathered once per object.
struct BeltBrimContext
{
    BeltFloorContext ctx;
    BeltBrimFrame    frame;
    // The brim ring and its apron, erBrim.
    BeltBrimRegion   ring;
    // The supports' first layer: a skin under every support foot, erSupportMaterial.
    BeltBrimRegion   skin;
    Flow             brim_flow;
    coord_t          pitch_u   = 0;
    coord_t          u_anchor  = 0;
    double           in_plane_pitch = 0.;   // mm
};

// Emit the cross-belt lines of region `r` that belong to the band [print_z - height, print_z].
// wall_inset is how far the centreline of the wall the band is levelled with (the object's
// outer wall, or the support line for the skin) lies inside where the layer's slice met
// the belt, mm.
static void belt_brim_band_paths(const BeltBrimContext      &bc,
                                 const BeltBrimRegion       &r,
                                 coordf_t                    print_z,
                                 coordf_t                    height,
                                 double                      wall_inset,
                                 const Polygons             &obstacles,
                                 ExtrusionEntityCollection  &out,
                                 ExPolygons                 &areas_out)
{
    if (r.empty())
        return;
    coordf_t u_lo = bc.ctx.cutoff_u(print_z - height);
    coordf_t u_hi = bc.ctx.cutoff_u(print_z);
    if (u_lo > u_hi)
        std::swap(u_lo, u_hi);

    // How wide this band is measured ON the belt, versus one nominal bead.
    const double band_in_plane = (u_hi - u_lo) * bc.frame.u_stretch();

    // Nozzle-to-belt clearance of the object's own outer wall where it rests on the belt.
    // The layer is sliced half a layer below print_z, so the slice meets the belt there, and
    // the wall's centreline is inset from that edge, where the belt lies lower by the inset
    // times the shear.  No brim line sits closer to the belt than this, so brim and object
    // are laid at the same height and one calibration of the belt height (endstop, extruder
    // offset) serves both.  A contour compensation that pushes the wall down to the belt
    // cannot be matched: the brim keeps a tenth of a layer, so its lines still have a
    // cross-section.
    const double wall_clearance = std::max(0.5 * height + wall_inset * std::abs(bc.frame.shear), 0.1 * height);

    std::vector<coord_t> us;
    double uniform_clearance = 0.;   // 0 => derive per line from its own position
    double line_pitch        = bc.in_plane_pitch;
    // One line also serves a band up to half a bead wider than the nominal pitch (a 0.3 mm
    // first layer at 45 degrees): its flow is matched to the band, so the bead is that much
    // wider. Two lattice lines in such a band would land almost on top of each other.
    if (band_in_plane <= 1.5 * bc.in_plane_pitch + EPSILON) {
        // Steep belt, which is the normal case: the band is narrower than one bead, so
        // exactly one line fits.  Place it at the outer wall's clearance rather than on
        // a nominal-spacing lattice.  On a lattice each line lands at an arbitrary point
        // in its band, the clearance sweeps [0, height] from band to band, and the bead
        // width therefore varies by 2x - visible as ragged, uneven brim lines.  A fixed
        // clearance is identical everywhere, so every bead is the same width.  It can
        // exceed the layer height, which only moves every band's line downhill by the
        // same distance.
        //
        // The spacing is then whatever the bands give (height / sin(tilt) on the belt)
        // rather than the nominal bead spacing, so the flow below is matched to THAT
        // pitch.  Matched flow at the real pitch is what keeps the sheet uniform and
        // gap-free; using nominal flow at band spacing would over-feed it.
        us.push_back(scale_(bc.ctx.cutoff_u(print_z - wall_clearance)));
        uniform_clearance = wall_clearance;
        line_pitch        = band_in_plane;
    } else {
        // Shallow belt: the band is wider than a bead, so it takes several lines and they
        // have to sit on the nominal lattice.  Their clearances then differ, and so do
        // their widths - unavoidable here, but shallow belts are the rare case.
        us = belt_brim_line_positions(scale_(u_lo), scale_(u_hi), bc.pitch_u, bc.u_anchor);
    }
    if (us.empty())
        return;

    // One lattice line at a time: the clearance - and therefore the extrusion
    // volume - is a property of the line's u, so the pieces of different lines
    // must not be pooled before the flow is resolved.
    // Overshoot the region so the clip, not the line's ends, decides the extent.
    const coord_t margin = coord_t(SCALED_EPSILON) + 1;
    coord_t       u_prev = std::numeric_limits<coord_t>::min();
    for (coord_t u : us) {
        // Nozzle-to-belt clearance for this line.  Constant along the line, because the
        // belt height depends only on the shear-axis coordinate.  Band-anchored lines
        // share one clearance by construction; lattice lines (shallow belts, or a first
        // layer thick enough that the band is wider than a bead) each get their own.
        //
        // A lattice line can fall where the belt is only a hair below the band's print_z.
        // The bead there would be laid scraping the belt while its flow is sized for a
        // taller cell, so it is moved to the outer wall's clearance, where the single-line
        // case puts every line.  (The clearance is along slice Z; the real gap under the
        // nozzle is clearance x cos(tilt).)
        double clearance = uniform_clearance;
        if (clearance <= 0.) {
            const Point probe = bc.frame.from_axis == 0 ? Point(u, 0) : Point(0, u);
            clearance = std::min(print_z - bc.ctx.floor_print_z(probe), height);
            if (clearance < wall_clearance) {
                clearance = wall_clearance;
                u         = scale_(bc.ctx.cutoff_u(print_z - clearance));
            }
        }
        // A line moved uphill can land on, or almost on, its neighbour; two beads closer
        // than half a pitch would be laid into the same cell.
        if (u_prev != std::numeric_limits<coord_t>::min() && std::abs(u - u_prev) < bc.pitch_u / 2)
            continue;
        u_prev = u;

        Polyline line;
        if (bc.frame.from_axis == 0)
            line.points = { Point(u, coord_t(r.bbox.min.y() - margin)),
                            Point(u, coord_t(r.bbox.max.y() + margin)) };
        else
            line.points = { Point(coord_t(r.bbox.min.x() - margin), u),
                            Point(coord_t(r.bbox.max.x() + margin), u) };

        Polylines pieces = intersection_pl(Polylines{ line }, r.polys);
        if (! obstacles.empty())
            pieces = diff_pl(pieces, obstacles);
        if (pieces.empty())
            continue;

        // A bead as tall as the clearance and line_pitch apart from the next fills exactly
        // one pitch x clearance cell of the sheet, at any clearance.  (with_cross_section()
        // makes a round bead of a quarter of the area asked for when the cell is small.)
        const float height_f = float(clearance);
        const Flow  f(Flow::rounded_rectangle_extrusion_width_from_spacing(float(line_pitch), height_f), height_f,
                      bc.brim_flow.nozzle_diameter());

        // Footprint of these beads, for the first-layer convex hull and bbox.
        for (const Polygon &p : offset(pieces, 0.5f * float(f.scaled_width())))
            areas_out.emplace_back(ExPolygon(p));

        extrusion_entities_append_paths(out.entities, chain_polylines(std::move(pieces)),
                                        r.role, f.mm3_per_mm(), f.width(), float(clearance));
    }
}

// Cut everything on `belt_side` out of `fills`, keeping each remaining piece's role and
// flow.  A loop or multi-path the cut touches comes back as the open paths that remain
// of it.  Pieces shorter than their own width are dropped: they would print as blobs on
// the skin bead that takes their place.
static void trim_support_fills_at_belt(ExtrusionEntityCollection &fills, const Polygons &belt_side)
{
    auto touched = [&belt_side](const Polyline &pl) { return ! intersection_pl(Polylines{ pl }, belt_side).empty(); };
    auto cut = [&belt_side](const ExtrusionPath &path, ExtrusionEntitiesPtr &dst) {
        Polylines pieces = diff_pl(Polylines{ path.polyline.to_polyline() }, belt_side);
        pieces.erase(std::remove_if(pieces.begin(), pieces.end(),
                                    [&path](const Polyline &pl) { return pl.length() < scale_(path.width); }),
                     pieces.end());
        extrusion_entities_append_paths(dst, std::move(pieces), path.role(), path.mm3_per_mm, path.width, path.height);
    };
    ExtrusionEntityCollection out;
    out.no_sort = fills.no_sort;
    for (const ExtrusionEntity *ee : fills.entities) {
        if (ee == nullptr)
            continue;
        if (const auto *sub = dynamic_cast<const ExtrusionEntityCollection *>(ee)) {
            ExtrusionEntityCollection copy = *sub;
            trim_support_fills_at_belt(copy, belt_side);
            if (! copy.empty())
                out.entities.emplace_back(new ExtrusionEntityCollection(std::move(copy)));
        } else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(ee)) {
            if (! touched(loop->polygon().split_at_first_point()))
                out.entities.emplace_back(ee->clone());
            else
                for (const ExtrusionPath &path : loop->paths)
                    cut(path, out.entities);
        } else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(ee)) {
            if (! touched(multi->as_polyline()))
                out.entities.emplace_back(ee->clone());
            else
                for (const ExtrusionPath &path : multi->paths)
                    cut(path, out.entities);
        } else if (const auto *path = dynamic_cast<const ExtrusionPath *>(ee)) {
            if (! touched(path->polyline.to_polyline()))
                out.entities.emplace_back(ee->clone());
            else
                cut(*path, out.entities);
        } else
            out.entities.emplace_back(ee->clone());
    }
    fills = std::move(out);
}

// Union of everything extruded at `print_z` that the brim must keep clear of, expressed
// in `self`'s local slicing frame.  Includes `self` itself: its slice at this Z can
// overhang outside the belt footprint and land in the brim ring, which the flattened
// brim_object_gap - a belt-plane separation - does not cover.
//
// SEQUENCING: this reads every object's layers and this object's own support layers.
// Another object's support step shifts that object's layer Z into the object frame for
// the duration of the run (PrintObject::_generate_support_material()), so the brims must
// not overlap with the parallel support step: Print::process() generates them one object
// after the other once that step is over (PrintObject::generate_belt_brim()), and an
// object that arrives on or leaves the plate invalidates the other brim owners' support
// step (PrintApply.cpp) so their brims are clipped against what is there now.  Only this
// object's supports are dodged; another object's support at the same Z is not.
// `region_bbox` bounds the brim; anything outside it cannot clip a brim line, so whole
// objects are skipped without materialising their polygons.  On a typical plate the
// objects do not overlap and every foreign object drops out here, which matters because
// this runs once per band - hundreds of times per object.
// `own_support` false leaves this object's supports out: the skin under them is laid
// where their toolpaths were cut away (make_belt_brim, step 1c), and the end caps of
// the cut lines would otherwise nick every bead.  `self_gap` (scaled) grows this object's
// own slices, so the skin keeps the support's first-layer XY distance from the part the
// way the generators keep the support itself (trim_support_layers_by_object).
static Polygons belt_brim_obstacles(const Print &print, const PrintObject &self,
                                    const BoundingBox &region_bbox, coordf_t print_z, coordf_t tol,
                                    bool own_support = true, coord_t self_gap = 0)
{
    const Point shift_self = self.instances().empty() ? Point(0, 0)
                                                      : self.instances().front().shift_without_plate_offset();
    Polygons out;
    for (const PrintObject *o : print.objects()) {
        const bool is_self = (o == &self);
        for (const PrintInstance &inst : o->instances()) {
            const Point delta = inst.shift_without_plate_offset() - shift_self;
            if (const Layer *l = o->get_layer_at_printz(print_z, tol)) {
                const coord_t grow = is_self ? self_gap : 0;
                BoundingBox   lb   = get_extents(l->lslices);
                lb.translate(delta.x(), delta.y());
                lb.offset(grow);
                if (lb.overlap(region_bbox)) {
                    Polygons ps = grow > 0 ? offset(l->lslices, float(grow)) : to_polygons(l->lslices);
                    for (Polygon &p : ps)
                        p.translate(delta);
                    polygons_append(out, std::move(ps));
                }
            }
            if (! is_self || ! own_support)
                continue;
            if (const SupportLayer *sl = o->get_support_layer_at_printz(print_z, tol)) {
                Polygons ps = sl->support_fills.polygons_covered_by_spacing();
                for (Polygon &p : ps)
                    p.translate(delta);
                polygons_append(out, std::move(ps));
            }
        }
    }
    if (out.size() < 2)
        return out;   // union_() of 0 or 1 polygons is pure overhead
    return union_(out);
}

void make_belt_brim(PrintObject &object)
{
    object.clear_belt_brim();
    if (! object.has_belt_brim())
        return;

    const Print &print = *object.print();
    BeltBrimContext bc;
    if (! bc.ctx.init(object.slicing_parameters(), print.config()))
        return;
    bc.frame = BeltBrimFrame{ bc.ctx.shear_factor(), bc.ctx.from_axis() };

    const size_t nlayers = object.layers().size();
    if (nlayers == 0)
        return;

    // 1. Belt footprint: the union of each layer's slice clipped to that layer's
    //    own contact band.  This is the object's bottom face, which on a belt is
    //    spread over every layer instead of sitting in layer 0.
    ExPolygons footprint_acc;
    // The first layer that touches the belt: where the leading-edge brim is cut.
    const Layer *first_contact = nullptr;
    // Per layer, the width of the outer wall resting on the belt, the narrowest where several
    // regions do, as it sits lowest; 0 where the layer does not rest on the belt.
    std::vector<double> contact_wall_width(nlayers, 0.);
    for (size_t i = 0; i < nlayers; ++ i) {
        const Layer &layer = *object.layers()[i];
        if (layer.lslices.empty())
            continue;
        // print_z - height, not the previous layer's print_z: variable layer
        // heights make the latter wrong.
        coordf_t u_lo = bc.ctx.cutoff_u(layer.print_z - layer.height);
        coordf_t u_hi = bc.ctx.cutoff_u(layer.print_z);
        if (u_lo > u_hi)
            std::swap(u_lo, u_hi);
        BoundingBox bb = get_extents(layer.lslices);
        bb.offset(scale_(1.));
        const Polygon band = band_box(bb, bc.frame.from_axis, u_lo, u_hi);
        if (band.empty())
            continue;
        ExPolygons contact = intersection_ex(layer.lslices, Polygons{ band });
        if (contact.empty())
            continue;
        if (first_contact == nullptr)
            first_contact = &layer;
        expolygons_append(footprint_acc, std::move(contact));
        for (const LayerRegion *layerm : layer.regions())
            if (! layerm->slices.empty() && ! intersection(to_polygons(layerm->slices.surfaces), Polygons{ band }).empty()) {
                const double w = layerm->flow(frExternalPerimeter).width();
                contact_wall_width[i] = contact_wall_width[i] > 0. ? std::min(contact_wall_width[i], w) : w;
            }
    }
    // 1b. Support feet.  Supports land on the belt wherever they reach it, ahead of or
    //     beside the part, and on a belt the generators give them none of the flat-bed
    //     first-layer treatment (SupportCommon.cpp, TreeSupport.cpp, TreeSupport3D.cpp:
    //     no expansion, no dense flange, no first-layer flow), so a foot starts as a
    //     sliver of base pattern at normal flow.  Each support layer's contact with the
    //     belt is taken like an object layer's; together they are the feet, which get a
    //     first layer of their own below (1c) and, with a brim, the ring around them.
    //     Where no object wall rests on the belt in a band the support line is the wall
    //     the ring is levelled with.  Always on a belt, whatever support_brim says: that
    //     option decides whether the flat-bed brim ring surrounds the supports, and on a
    //     belt the ring is the lesser part of what keeps a foot down.  The leading-edge
    //     cut below stays at the object's first contact: a foot ahead of it is downhill
    //     and kept.
    ExPolygons   feet_acc;
    const double support_wall_width = object.support_layers().empty() ? 0. : support_material_flow(&object).width();
    {
        const LayerPtrs &object_layers = object.layers();
        for (const SupportLayer *sl : object.support_layers()) {
            if (sl->support_fills.empty() || sl->height < EPSILON)
                continue;
            const Polygons fills = sl->support_fills.polygons_covered_by_spacing();
            if (fills.empty())
                continue;
            coordf_t u_lo = bc.ctx.cutoff_u(sl->print_z - sl->height);
            coordf_t u_hi = bc.ctx.cutoff_u(sl->print_z);
            if (u_lo > u_hi)
                std::swap(u_lo, u_hi);
            BoundingBox bb = get_extents(fills);
            bb.offset(scale_(1.));
            const Polygon band = band_box(bb, bc.frame.from_axis, u_lo, u_hi);
            if (band.empty())
                continue;
            ExPolygons contact = intersection_ex(fills, Polygons{ band });
            if (contact.empty())
                continue;
            expolygons_append(feet_acc, std::move(contact));
            // The object layer whose band this is.  Support layers share the object's
            // print_z lattice unless they are independent, so look the band up by Z.
            auto it = std::lower_bound(object_layers.begin(), object_layers.end(), sl->print_z - 0.5 * sl->height,
                                       [](const Layer *l, coordf_t z) { return l->print_z < z; });
            if (it != object_layers.end()) {
                const size_t i = size_t(it - object_layers.begin());
                if (contact_wall_width[i] <= 0.)
                    contact_wall_width[i] = support_wall_width;
            }
        }
    }
    const ExPolygons feet = union_ex(feet_acc);

    // 1c. The supports' first layer.  On a flat bed it is the flange: the support's
    //     footprint widened by raft_first_layer_expansion, dense, slow.  On the belt that
    //     layer is spread over every band, so it is laid the way the brim is: one bead per
    //     band across the whole foot at the support line's clearance, flow matched to the
    //     band pitch (belt_brim_band_paths), which tiles the belt gap-free.  The region is
    //     the feet widened by the expansion in the true on-belt metric, so a tree tip
    //     lands on a bead as wide as the expansion instead of on its own sliver, and the
    //     bands ahead of a foot are laid before the foot arrives, as the apron is.
    //
    //     The supports' own toolpaths are then cut clear of that band: the bead takes
    //     their place, and the layer above rests on it with a full layer under it, the
    //     way layer two rests on the flange.  Done here, after the generators, rather than
    //     in their three belt clips: it is the toolpaths that have to clear the band, the
    //     generators work in different frames (global, local, sheared by a support lean),
    //     and the feet have to be measured before the cut.  The support step always
    //     re-runs before this (PrintObject::generate_belt_brim), so the fills are never
    //     cut twice.  The bands' lines print with the brim filament (belt_brim_filament),
    //     not the support filament.
    const PrintObjectConfig &cfg = object.config();
    ExPolygons skin_flat;
    if (! feet.empty()) {
        const double expansion = std::max(0., cfg.raft_first_layer_expansion.value);
        skin_flat = offset_ex(belt_flatten(feet, bc.frame), float(scale_(expansion)), jtRound, SCALED_RESOLUTION);
        bc.skin.set(belt_unflatten(skin_flat, bc.frame), erSupportMaterial);
        for (SupportLayer *sl : object.support_layers()) {
            if (sl->support_fills.empty() || sl->height < EPSILON)
                continue;
            const coordf_t keep = belt_support_keep_z(sl->height, support_wall_width, bc.frame.shear);
            trim_support_fills_at_belt(sl->support_fills, bc.ctx.surface_polygon(sl->print_z - keep));
        }
    }

    ExPolygons footprint = footprint_acc;
    expolygons_append(footprint, feet);
    footprint = union_ex(footprint);
    if (footprint.empty())
        return;

    // 2. Brim region, offset in the flattened (true on-belt) metric, around the object's
    //    contact and the supports' skin.
    bc.brim_flow = print.brim_flow();
    const double  flow_w = bc.brim_flow.scaled_spacing() * SCALING_FACTOR;
    // Quantize to an even number of lines, as the plate brim does.
    const coord_t width   = scale_(std::floor(cfg.brim_width.value / flow_w / 2) * flow_w * 2);
    const coord_t leading = scale_(cfg.leading_brim_length.value);
    const coord_t lateral = scale_(cfg.extra_brim_width.value);
    const coord_t gap     = scale_(cfg.brim_object_gap.value);

    // Belt printers collapse Auto / Mouse ear / Painted to outer-only: the auto width
    // heuristic and flat ear discs have no meaning on a tilted plane.  Leading-edge-only
    // is an outer brim too; it is narrowed down to the first contact below.
    const BrimType bt = cfg.brim_type.value;
    const bool has_outer = bt == btOuterOnly || bt == btOuterAndInner
                        || bt == btAutoBrim  || bt == btEar || bt == btPainted
                        || bt == btLeadingEdgeOnly;
    const bool has_inner = bt == btInnerOnly || bt == btOuterAndInner;
    // Mirrors the brim half of PrintObject::has_belt_brim(): the other half, supports on
    // the belt, wants the skin and no ring.
    const bool wants_ring = (has_outer || has_inner)
                         && (has_outer ? (width > 0 || leading > 0 || lateral > 0) : width > 0);

    if (wants_ring) {
        ExPolygons base_flat = belt_flatten(footprint, bc.frame);
        expolygons_append(base_flat, skin_flat);
        ExPolygons ring = belt_unflatten(
            belt_brim_region(union_ex(base_flat), has_outer, has_inner, width, gap, leading, lateral, bc.frame),
            bc.frame);
        if (bt == btLeadingEdgeOnly && first_contact != nullptr && ! ring.empty())
            // The cut is the uphill edge of the first contact's band: everything past it
            // belongs to later contacts.  The first contact is the first layer that touches
            // the belt (step 1), neither layers().front(), an empty lead-in layer, nor the
            // first layer with geometry, which is an overhang's tip when the part overhangs
            // its leading end: both lie ahead of the part.
            ring = belt_brim_clip_leading_edge(ring, bc.frame, bc.ctx.cutoff_u(first_contact->print_z));
        bc.ring.set(std::move(ring), erBrim);
    }

    if (bc.ring.empty() && bc.skin.empty())
        return;
    BoundingBox region_bbox = bc.ring.bbox;
    region_bbox.merge(bc.skin.bbox);

    // 3. Line lattice.  Fixed pitch in the flattened metric, anchored at the
    //    footprint's leading-most edge so lines stay collinear across
    //    disconnected islands and across the apron prologue.
    bc.pitch_u = std::max<coord_t>(1, coord_t(bc.brim_flow.scaled_spacing() * bc.frame.cos_tilt()));
    bc.in_plane_pitch = unscale<double>(bc.pitch_u) * bc.frame.u_stretch();
    {
        const BoundingBox fbb = get_extents(footprint);
        const bool low_side = bc.frame.shear > 0.;
        bc.u_anchor = bc.frame.from_axis == 0 ? (low_side ? fbb.min.x() : fbb.max.x())
                                              : (low_side ? fbb.min.y() : fbb.max.y());
    }

    // How far each layer's outer wall, which the layer's brim band is levelled with, lies
    // inside where the layer's slice met the belt: half its width, less the XY contour
    // compensation that grew the slice toward the belt (none where slicing skips it, see
    // PrintObject::slice_volumes()).  The first layer's walls take initial_layer_line_width.
    // A layer that does not rest on the belt takes the wall of the last one before it that
    // does, and the layers ahead of the part the first one's.
    const double contour_compensation = print.config().filament_diameter.size() > 1 && object.is_mm_painted() ?
        0. : object.config().xy_contour_compensation.value;
    std::vector<double> wall_insets(nlayers);
    {
        const auto first_wall = std::find_if(contact_wall_width.begin(), contact_wall_width.end(), [](double w) { return w > 0.; });
        double     width      = first_wall != contact_wall_width.end() ? *first_wall : 0.;
        for (size_t i = 0; i < nlayers; ++ i) {
            if (contact_wall_width[i] > 0.)
                width = contact_wall_width[i];
            wall_insets[i] = 0.5 * width - contour_compensation;
        }
    }
    // The skin is levelled with the support line it replaces, in every band: the same
    // clearance belt_support_keep_z() cleared the toolpaths down to.
    const double  skin_inset = 0.5 * support_wall_width;
    const coord_t skin_gap   = scale_(std::max(0., cfg.support_object_first_layer_gap.value));

    // Both regions' lines for one band.  The skin first: it is the first layer under
    // the support, the ring only surrounds it.
    auto emit_band = [&](coordf_t print_z, coordf_t height, double ring_inset,
                         ExtrusionEntityCollection &out, ExPolygons &areas_out) {
        if (! bc.skin.empty()) {
            const Polygons obstacles = belt_brim_obstacles(print, object, bc.skin.bbox, print_z, 0.5 * height,
                                                           /*own_support=*/false, skin_gap);
            belt_brim_band_paths(bc, bc.skin, print_z, height, skin_inset, obstacles, out, areas_out);
        }
        if (! bc.ring.empty()) {
            const Polygons obstacles = belt_brim_obstacles(print, object, bc.ring.bbox, print_z, 0.5 * height);
            belt_brim_band_paths(bc, bc.ring, print_z, height, ring_inset, obstacles, out, areas_out);
        }
    };

    // 4. Bands coincident with an object layer.
    std::vector<ExtrusionEntityCollection> by_layer(nlayers);
    std::vector<ExPolygons>                areas_by_layer(nlayers);
    for (size_t i = 0; i < nlayers; ++ i) {
        const Layer &layer = *object.layers()[i];
        emit_band(layer.print_z, layer.height, wall_insets[i], by_layer[i], areas_by_layer[i]);
    }

    // 5. Apron prologue: the part of the region downhill of the object's first
    //    layer, which has no object layer to ride on.  It continues the first layer's
    //    band, so it takes that layer's height, and the first contact's wall.
    std::vector<BeltBrimBand> prologue;
    {
        const Layer   &first  = *object.layers().front();
        const coordf_t h      = first.height;
        const bool     low_side = bc.frame.shear > 0.;
        const coord_t  u_lead_s = bc.frame.from_axis == 0
            ? (low_side ? region_bbox.min.x() : region_bbox.max.x())
            : (low_side ? region_bbox.min.y() : region_bbox.max.y());
        const coordf_t u_lead  = unscale<double>(u_lead_s);
        // print_z at which the belt surface crosses the region's leading edge.
        const coordf_t z_lead  = bc.ctx.shear_factor() * u_lead
                               + bc.ctx.floor_offset() + bc.ctx.z_shift();
        if (h > EPSILON)
            for (coordf_t z = first.print_z - h; z > z_lead - h; z -= h) {
                BeltBrimBand band;
                band.print_z = z;
                band.height  = h;
                emit_band(z, h, wall_insets.front(), band.fills, band.areas);
                if (! band.fills.empty())
                    prologue.emplace_back(std::move(band));
            }
        // Lowest Z first, so collect_layers_to_print sees them in print order.
        std::reverse(prologue.begin(), prologue.end());
    }

    object.set_belt_brim(std::move(by_layer), std::move(areas_by_layer), std::move(prologue));
}

} // namespace Slic3r
