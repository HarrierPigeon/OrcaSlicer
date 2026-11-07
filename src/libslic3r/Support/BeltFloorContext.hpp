#pragma once

#include "../libslic3r.h"
#include "../Point.hpp"
#include "../Polygon.hpp"
#include "../Slicing.hpp"
#include "../PrintConfig.hpp"
#include "SupportDirection.hpp"
#include <cstddef>
#include <functional>
#include <vector>

namespace Slic3r {

class PrintObject;

// Belt floor context: encapsulates the parameters and polygon computation
// for belt printer floor clipping in support generation.
//
// All belt floor code across SupportMaterial, TreeSupport, TreeSupport3D,
// and TreeModelVolumes uses the same 4 parameters and the same 4-case
// polygon construction.  This class consolidates that logic.
//
// Construct once per PrintObject, then call surface_polygon() or
// valid_region_polygon() per layer with the layer's print_z.
//
// While an object's support leans (Support/SupportDirection.hpp) its layers are held in a sheared
// frame; set_lean() makes every query here answer in that frame. Without it, or with no lean, the
// formulas reduce exactly to the plain ones.
class BeltFloorContext
{
public:
    BeltFloorContext() = default;

    // Initialize from slicing parameters and print config.
    // Uses global Z coordinates (for SupportMaterial, non-organic TreeSupport).
    bool init(const SlicingParameters &sp, const PrintConfig &pcfg);

    // Initialize with a Z offset subtracted from z_shift.
    // Uses local Z coordinates (for TreeSupport3D, TreeModelVolumes organic pipeline).
    bool init_local(const SlicingParameters &sp, const PrintConfig &pcfg,
                    double global_z_offset);

    // Re-express the floor in the sheared support-lean frame. z_offset_to_global is what to add
    // to the print_z this context is queried with to get the global print_z: 0 for a caller in
    // global print_z, PrintObject::belt_global_z_offset() for one in the object-local frame.
    void set_lean(const SupportLean &lean, double z_offset_to_global);

    bool is_active() const { return m_active; }

    // Compute the belt-side half-plane polygon at a given print_z.
    // This is the region where the belt surface exists.
    Polygons surface_polygon(coordf_t print_z) const;

    // Compute the valid-region half-plane polygon at a given print_z.
    // This is the complement: the region where support is allowed.
    Polygons valid_region_polygon(coordf_t print_z) const;

    // Compute the belt floor Z position at a given XY position (in slicing coords;
    // in the sheared frame once set_lean() was called).
    // Returns -infinity if not active.
    double floor_print_z(const Point &pos_slicing) const;

    // The from_axis coordinate (unscaled, slicing frame) where the belt surface
    // crosses a horizontal plane at print_z.  Inverse of floor_print_z() along
    // the shear axis.  Only meaningful when is_active().
    coordf_t cutoff_u(coordf_t print_z) const
        { return (print_z - m_z_shift - m_floor_offset) / m_shear_factor - (print_z + m_lean_zoff) * m_lean_slope_u; }

    // Pre-compute belt floor polygons for a range of layers.
    // layer_print_z(i) returns the print_z for layer index i.
    std::vector<Polygons> compute_per_layer_floors(
        size_t num_layers,
        const std::function<double(size_t)> &layer_print_z) const;

    // Accessors
    // Slope of the floor along from_axis in the un-sheared slicing frame.
    double shear_factor()  const { return m_shear_factor; }
    // Slope of the floor along from_axis in the frame the context is queried in:
    // shear_factor() without a lean, shear / (1 - shear * slope_u) with one.
    double effective_shear_factor() const { return m_shear_factor / this->lean_denominator(); }
    int    from_axis()     const { return m_from_axis; }
    double z_shift()       const { return m_z_shift; }
    double floor_offset()  const { return m_floor_offset; }

private:
    bool   m_active       = false;
    double m_shear_factor = 0.0;
    int    m_from_axis    = 1;  // 0=X, 1=Y
    double m_z_shift      = 0.0;
    double m_floor_offset = 0.0;
    // Support lean, see set_lean(). Zero slope = the plain slicing frame. m_lean_zoff maps a
    // queried print_z onto the lean's own z: print_z + m_lean_zoff - z_origin.
    double m_lean_slope_u = 0.0;
    double m_lean_zoff    = 0.0;

    // 1 - shear * slope_u, how fast a column along the lean closes on the floor; support_lean()
    // never admits a lean for which it is not positive.
    double lean_denominator() const;

    // Internal: compute the raw half-plane polygon.
    // If belt_surface=true, returns the belt side; otherwise the valid (complement) side.
    Polygons half_plane(coordf_t print_z, bool belt_surface) const;
};

} // namespace Slic3r
