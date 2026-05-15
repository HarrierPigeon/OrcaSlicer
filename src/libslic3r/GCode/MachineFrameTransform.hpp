#ifndef slic3r_MachineFrameTransform_hpp_
#define slic3r_MachineFrameTransform_hpp_

#include "../libslic3r.h"
#include "../Point.hpp"
#include "../PrintConfig.hpp"

namespace Slic3r {

// Post-stage machine-frame transform for belt printers.
//
// Applied in BeltGCodeWriter::to_machine_coords AFTER the back-transform
// and the existing gcode_remap_* axis remap, and BEFORE per-axis origin snap.
// Maps Cartesian (axis-permuted) G-code coordinates into the printer's
// physical machine frame using a parallel set of options:
//   gcode_shear_x/y/z + _angle + _from
//   gcode_scale_x/y/z + _angle
//   post_gcode_remap_x/y/z
//
// Composition matches the mesh-side pipeline: shear * scale * post_remap.
class MachineFrameTransform
{
public:
    MachineFrameTransform() = default;

    // Initialize from belt printer config.  Returns true if a non-identity
    // transform was computed.  Inactive when belt_printer is disabled or
    // all three sub-stages are identity.
    bool init_from_config(const PrintConfig &config);

    // Apply the transform to a point.  Returns pos unchanged if not active.
    Vec3d apply(const Vec3d &pos) const;

    bool is_active() const { return m_active; }

private:
    bool        m_active    = false;
    Transform3d m_transform = Transform3d::Identity();
};

} // namespace Slic3r

#endif // slic3r_MachineFrameTransform_hpp_
