#pragma once

// Support direction: growing support along a direction other than the layer normal.
//
// The generators can only stack support straight along the slice frame's Z, so to lean it along a
// unit direction u the layers are sheared instead: every layer's geometry is translated by
// -(z - z_origin) * (u.xy / u.z), a column along u becomes vertical, the unmodified generator runs,
// and everything is translated back at its own print_z. Overhangs are therefore judged against u,
// and the belt floor (BeltFloorContext) is seen through the same shear. The design is in
// docs/HLSD/support-direction.md.

#include "../libslic3r.h"
#include "../Geometry.hpp"
#include "../Point.hpp"

#include <cmath>

namespace Slic3r {

class PrintObject;
class PrintObjectConfig;

// The lean in effect for one object's support generation.
struct SupportLean
{
    // XY drift of a support column per unit of slice-frame z, i.e. u.xy / u.z. Zero = follow layers.
    Vec2d  slope    { 0., 0. };
    // The print_z the shear is measured from: no shift there, growing with the height above it.
    // The object's belt_global_z_offset, so on a belt the shift follows the object-local height.
    double z_origin { 0. };
    // The configured direction exceeded the object's max lean and was pulled back to it.
    bool   clamped  { false };
    // On a belt, the configured direction lies along or beyond the belt plane, so a column along
    // it would never meet the belt; the support follows the layers instead.
    bool   floor_unreachable { false };

    // Past this lean a support column overhangs itself beyond what prints reliably; the option
    // allows more, with a warning.
    static constexpr double safe_lean_deg() { return 45.; }

    bool        active() const { return slope.squaredNorm() > 1e-18; }
    // The lean actually in effect, in degrees away from the layer normal.
    double      lean_deg() const { return Geometry::rad2deg(std::atan(slope.norm())); }
    // Scaled translation that shears geometry lying at the GLOBAL print_z (Layer::print_z as sliced,
    // belt offset included) into the frame where the lean is vertical. The inverse is the negated
    // point, exactly. Object layers are moved by PrintObject::support_lean_shift(), which records
    // the shift each one received, so readers need not know which frame print_z is in.
    Point       shift_at(double print_z) const
        { return Point::new_scale(- (print_z - z_origin) * slope.x(), - (print_z - z_origin) * slope.y()); }
};

// The configured support direction as a unit vector in the bed frame (the frame the object is
// placed in, before any belt rotation), or a zero vector for "follow layers".
Vec3d support_direction_bed_frame(const PrintObjectConfig &cfg);

// The lean the object's config asks for, resolved into its slice frame. Inactive for the default
// "follow layers", and for any direction that coincides with the layer normal.
SupportLean support_lean(const PrintObject &object);

// Holds the object's layers in the sheared frame for the lifetime of the scope and puts everything
// back -- layers and the support layers generated meanwhile -- when it ends, including on an
// exception. Constructing it on an object whose lean is inactive is a no-op.
class ScopedSupportShear
{
public:
    explicit ScopedSupportShear(PrintObject &object);
    ~ScopedSupportShear();
    ScopedSupportShear(const ScopedSupportShear &) = delete;
    ScopedSupportShear& operator=(const ScopedSupportShear &) = delete;

    bool   active() const            { return m_lean.active(); }
    bool   clamped() const           { return m_lean.clamped; }
    bool   floor_unreachable() const { return m_lean.floor_unreachable; }
    double lean_deg() const          { return m_lean.lean_deg(); }

private:
    PrintObject &m_object;
    SupportLean  m_lean;
};

} // namespace Slic3r
