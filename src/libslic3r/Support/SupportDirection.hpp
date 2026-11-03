#pragma once

// Support direction: growing support along a direction other than the layer normal.
//
// A layer-based support generator can only stack support straight along the slice frame's Z. To
// lean it along another unit direction u (slice frame, u.z > 0) without touching the three
// generators, the whole problem is sheared instead: every layer's geometry is translated by
// -z * (u.xy / u.z), which maps a column along u onto a vertical column. The unmodified generator
// then runs against that sheared object, and everything it produced is translated back by
// +z * (u.xy / u.z) at the layer's own print_z. A shear preserves z, so layers stay layers and an
// object/support contact at a given z stays a contact.
//
// What follows from that choice:
//   * overhang detection runs in the sheared frame too, so a face is judged against the support
//     direction rather than the layer normal. This is the same thing the build-plate tilt shift in
//     the generators does for a belt; while a lean is active the generators skip that shift so the
//     tilt isn't counted twice;
//   * the belt floor (BeltFloorContext) is re-expressed through the same shear, so a leaning column
//     still terminates on the belt;
//   * the lean is limited to the object's support_direction_max_lean: a leaning column overhangs
//     itself by exactly the lean angle, so past safe_lean_deg() it needs tuning and test prints,
//     and generation warns. A direction past the limit keeps its azimuth and is pulled back to
//     the limit (a direction pointing straight down has no azimuth and falls back to following
//     the layers).
//
// "Vertical" means +Z of the bed frame the object was placed in. On a belt printer the object is
// rotated by the belt angle before slicing (BeltSliceStrategy), so that direction is the belt
// normal in the slice frame and the support grows perpendicular to the belt. On a flat bed it is
// the layer normal already and the lean is inactive.
//
// Ported from Unlayered-Slicer (Support/SupportDirection.hpp, commits fe1ea365be and 96bd8d44ec).

#include "../libslic3r.h"
#include "../Geometry.hpp"
#include "../Point.hpp"

#include <cmath>

namespace Slic3r {

class PrintObject;
class PrintObjectConfig;
class PrintConfig;

// The lean in effect for one object's support generation.
struct SupportLean
{
    // XY drift of a support column per unit of slice-frame z, i.e. u.xy / u.z. Zero = follow layers.
    Vec2d slope   { 0., 0. };
    // The configured direction exceeded the object's max lean and was pulled back to it.
    bool  clamped { false };

    // Past this lean a support column overhangs itself beyond what prints reliably; the option
    // allows more, with a warning.
    static constexpr double safe_lean_deg() { return 45.; }

    bool        active() const { return slope.squaredNorm() > 1e-18; }
    // The lean actually in effect, in degrees away from the layer normal.
    double      lean_deg() const { return Geometry::rad2deg(std::atan(slope.norm())); }
    // Scaled translation that shears geometry lying at print_z into the frame where the lean is
    // vertical. The inverse is the negated point, exactly, so a round trip loses nothing.
    // print_z is Layer::print_z as stored on the object layers when ScopedSupportShear was
    // constructed: on a belt that includes PrintObject::belt_global_z_offset(). Code that works
    // in a local frame (the normal generator's run, the organic tree pipeline) has to add that
    // offset back before calling this.
    Point       shift_at(double print_z) const { return Point::new_scale(- print_z * slope.x(), - print_z * slope.y()); }
    // The same shear as an affine map on unscaled slice-frame coordinates.
    Transform3d shear() const;
};

// The configured support direction as a unit vector in the bed frame (the frame the object is
// placed in, before any belt rotation), or a zero vector for "follow layers". Shared by the
// slicer (support_lean) and the 3D scene (direction arrow).
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

    bool   active() const   { return m_lean.active(); }
    bool   clamped() const  { return m_lean.clamped; }
    double lean_deg() const { return m_lean.lean_deg(); }

private:
    PrintObject &m_object;
    SupportLean  m_lean;
};

} // namespace Slic3r
