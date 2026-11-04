#include "SupportDirection.hpp"

#include "../Geometry.hpp"
#include "../Layer.hpp"
#include "../Point.hpp"
#include "../Print.hpp"
#include "../PrintConfig.hpp"
#include "../libslic3r.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace Slic3r {

Transform3d SupportLean::shear() const
{
    Transform3d t = Transform3d::Identity();
    t.linear()(0, 2) = - slope.x();
    t.linear()(1, 2) = - slope.y();
    return t;
}

Vec3d support_direction_bed_frame(const PrintObjectConfig &cfg)
{
    switch (cfg.support_direction.value) {
    case sdVertical:
        return Vec3d::UnitZ();
    case sdCustom:
        return Geometry::tilt_up_direction(cfg.support_direction_tilt_x.value, cfg.support_direction_tilt_y.value);
    case sdLayers:
    default:
        return Vec3d::Zero();
    }
}

SupportLean support_lean(const PrintObject &object)
{
    SupportLean              lean;
    const PrintObjectConfig &cfg   = object.config();
    const Vec3d              u_bed = support_direction_bed_frame(cfg);
    if (u_bed.squaredNorm() < 1e-18)
        return lean;

    // Bed frame -> slice frame. trafo_sliced() is the belt pre-slice transforms applied on top of
    // trafo_centered(), so their quotient is exactly that extra rotation: identity on a flat bed,
    // the belt rotation on a belt printer. The instance's own scale and mirror cancel out. A
    // direction (not a normal) maps through the linear part itself; it is normalised afterwards in
    // case any scale residue survived.
    const Eigen::Matrix3d R = (object.trafo_sliced() * object.trafo_centered().inverse()).linear();
    Vec3d                 u = R * u_bed;
    if (u.squaredNorm() < 1e-18)
        return lean;
    u.normalize();
    // The direction names a LINE for the column, not which way along it: support always grows
    // towards +Z of the slice frame, from the floor up.
    if (u.z() < 0.)
        u = - u;

    const double max_lean = std::clamp(cfg.support_direction_max_lean.value, 0., 85.);
    const double max_cos  = std::cos(Geometry::deg2rad(max_lean));
    if (u.z() < max_cos - EPSILON) {
        // Too far from the layer normal: keep the azimuth, pull the tilt back to the limit. A
        // direction lying in the layer plane (rxy == 1) is pulled back the same way; one exactly
        // along the normal never reaches this branch.
        lean.clamped = true;
        const double rxy = std::hypot(u.x(), u.y());
        const double s   = std::sin(Geometry::deg2rad(max_lean));
        u = Vec3d(u.x() / rxy * s, u.y() / rxy * s, max_cos);
    }
    lean.slope = Vec2d(u.x() / u.z(), u.y() / u.z());
    if (! lean.active())
        // Coincides with the layer normal (within the activity threshold): follow the layers.
        lean = SupportLean();
    return lean;
}

// Layer::print_z is read at construction and again at destruction. Inside
// PrintObject::_generate_support_material() the normal generator temporarily subtracts
// PrintObject::m_belt_global_z_offset from every object layer's print_z for its run and adds it
// back onto the object layers and the new support layers before returning, so by the time this
// scope ends every print_z is global again and the shifts applied here and in the destructor
// cancel exactly (the tree generators never touch print_z, organic support gets the offset added
// onto its new layers the same way). shift_at() is therefore always evaluated on global print_z.
ScopedSupportShear::ScopedSupportShear(PrintObject &object)
    : m_object(object), m_lean(support_lean(object))
{
    if (! m_lean.active())
        return;
    m_object.m_support_lean = m_lean;
    for (Layer *layer : m_object.m_layers)
        layer->translate(m_lean.shift_at(layer->print_z));
}

ScopedSupportShear::~ScopedSupportShear()
{
    if (! m_lean.active())
        return;
    for (Layer *layer : m_object.m_layers)
        layer->translate(- m_lean.shift_at(layer->print_z));
    // Everything the generator built lives in the sheared frame; its own print_z (which may differ
    // from any object layer's, e.g. raft layers) says exactly how far it moved.
    for (SupportLayer *layer : m_object.m_support_layers)
        layer->translate(- m_lean.shift_at(layer->print_z));
    m_object.m_support_lean = SupportLean();
}

} // namespace Slic3r
