#include "BeltFloorContext.hpp"
#include "../Point.hpp"
#include "../Polygon.hpp"
#include "../PrintConfig.hpp"
#include "../Slicing.hpp"
#include "../libslic3r.h"

#include <cmath>
#include <limits>
#include <cstddef>
#include <functional>
#include <vector>

namespace Slic3r {

bool BeltFloorContext::init(const SlicingParameters &sp, const PrintConfig &pcfg)
{
    m_active = false;
    m_shear_factor = sp.belt_floor_shear_factor;
    m_from_axis    = sp.belt_floor_from_axis;
    m_z_shift      = sp.belt_floor_z_shift;
    m_floor_offset = pcfg.belt_support_floor_offset.value;
    m_lean_slope_u = 0.0;
    m_lean_zoff    = 0.0;

    if (std::abs(m_shear_factor) < EPSILON)
        return false;

    m_active = true;
    return true;
}

bool BeltFloorContext::init_local(const SlicingParameters &sp, const PrintConfig &pcfg,
                                  double global_z_offset)
{
    if (!init(sp, pcfg))
        return false;
    // Local Z: subtract the global Z offset so polygon computation
    // works in the object's local coordinate space.
    m_z_shift -= global_z_offset;
    return true;
}

void BeltFloorContext::set_lean(const Vec2d &slope_unscaled_per_mm, double z_offset_to_global)
{
    m_lean_slope_u = m_from_axis == 0 ? slope_unscaled_per_mm.x() : slope_unscaled_per_mm.y();
    m_lean_zoff    = z_offset_to_global;
}

double BeltFloorContext::lean_denominator() const
{
    const double d = 1.0 - m_shear_factor * m_lean_slope_u;
    if (std::abs(d) >= 1e-6)
        return d;
    return d < 0. ? -1e-6 : 1e-6;
}

Polygons BeltFloorContext::surface_polygon(coordf_t print_z) const
{
    return half_plane(print_z, /*belt_surface=*/true);
}

Polygons BeltFloorContext::valid_region_polygon(coordf_t print_z) const
{
    return half_plane(print_z, /*belt_surface=*/false);
}

double BeltFloorContext::floor_print_z(const Point &pos_slicing) const
{
    if (!m_active)
        return -std::numeric_limits<double>::infinity();
    double pos = unscale<double>(m_from_axis == 0 ? pos_slicing.x() : pos_slicing.y());
    // Sheared frame: pos is u' = u - (z + zoff) * slope_u, so the floor equation
    // z = shear * u + off + z_shift becomes
    //   z = shear * (u' + (z + zoff) * slope_u) + off + z_shift
    // and solving for z gives the expression below. Without a lean slope_u = 0 and
    // this is the plain z = shear * u + off + z_shift.
    return (m_shear_factor * (pos + m_lean_zoff * m_lean_slope_u) + m_floor_offset + m_z_shift) / this->lean_denominator();
}

std::vector<Polygons> BeltFloorContext::compute_per_layer_floors(
    size_t num_layers,
    const std::function<double(size_t)> &layer_print_z) const
{
    std::vector<Polygons> result(num_layers);
    if (!m_active)
        return result;
    for (size_t i = 0; i < num_layers; ++i)
        result[i] = surface_polygon(layer_print_z(i));
    return result;
}

Polygons BeltFloorContext::half_plane(coordf_t print_z, bool belt_surface) const
{
    if (!m_active)
        return {};

    const double cutoff = this->cutoff_u(print_z);
    const coord_t cutoff_scaled = scale_(cutoff);
    const coord_t large_bound   = scale_(1e3);

    // The belt surface is on one side of the cutoff line; the valid region
    // is on the other side.  Which side depends on the sign of the floor's
    // slope in the frame the polygon is built in (effective_shear_factor():
    // the raw shear without a lean, and the same sign as long as the lean
    // points up and away from the belt).
    //
    // belt_surface=true  → the belt side (where support should NOT exist)
    // belt_surface=false → the valid side (where support IS allowed)
    //
    // For slope > 0: belt surface is from_axis >= cutoff
    // For slope < 0: belt surface is from_axis <= cutoff
    bool high_side = (this->effective_shear_factor() > 0) == belt_surface;

    Polygon poly;
    if (m_from_axis == 0) {
        if (high_side) {
            // X >= cutoff
            poly.points = {
                Point(cutoff_scaled, -large_bound),
                Point(large_bound,   -large_bound),
                Point(large_bound,    large_bound),
                Point(cutoff_scaled,  large_bound)
            };
        } else {
            // X < cutoff
            poly.points = {
                Point(-large_bound,  -large_bound),
                Point(cutoff_scaled, -large_bound),
                Point(cutoff_scaled,  large_bound),
                Point(-large_bound,   large_bound)
            };
        }
    } else {
        if (high_side) {
            // Y >= cutoff
            poly.points = {
                Point(-large_bound, cutoff_scaled),
                Point( large_bound, cutoff_scaled),
                Point( large_bound, large_bound),
                Point(-large_bound, large_bound)
            };
        } else {
            // Y < cutoff
            poly.points = {
                Point(-large_bound, -large_bound),
                Point( large_bound, -large_bound),
                Point( large_bound, cutoff_scaled),
                Point(-large_bound, cutoff_scaled)
            };
        }
    }
    return { poly };
}

} // namespace Slic3r
