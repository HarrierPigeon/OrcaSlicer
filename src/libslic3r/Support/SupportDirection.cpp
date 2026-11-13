#include "SupportDirection.hpp"

#include "../Geometry.hpp"
#include "../Layer.hpp"
#include "../Point.hpp"
#include "../Print.hpp"
#include "../PrintConfig.hpp"
#include "../Slicing.hpp"
#include "../libslic3r.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace Slic3r {

Vec3d world_up_bed_frame(const PrintConfig &print_config)
{
    return print_config.belt_printer.value ?
        Vec3d::UnitZ() :
        Geometry::tilt_up_direction(print_config.build_plate_tilt_x.value, print_config.build_plate_tilt_y.value);
}

Vec3d support_direction_bed_frame(const PrintObjectConfig &cfg, const Vec3d &world_up)
{
    switch (cfg.support_direction.value) {
    case sdVertical:
        return world_up;
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
    const Vec3d              u_bed = support_direction_bed_frame(cfg, world_up_bed_frame(object.print()->config()));
    // A raft alone is not leaned.
    if (u_bed.squaredNorm() < 1e-18 || ! object.has_support())
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

    lean.z_origin = object.belt_global_z_offset();
    const double max_lean = std::clamp(cfg.support_direction_max_lean.value, 0., 85.);
    const double rxy      = std::hypot(u.x(), u.y());
    if (Geometry::rad2deg(std::atan2(rxy, u.z())) > max_lean + 1e-9) {
        // Too far from the layer normal: keep the heading, pull the tilt back to the limit. rxy > 0
        // here because a direction exactly along the normal never reaches this branch.
        lean.clamped = true;
        const double s = std::sin(Geometry::deg2rad(max_lean));
        u = Vec3d(u.x() / rxy * s, u.y() / rxy * s, std::cos(Geometry::deg2rad(max_lean)));
    }
    lean.slope = Vec2d(u.x() / u.z(), u.y() / u.z());

    // A column that leans along the belt plane, or past it, never meets the belt: the floor would
    // sit infinitely far down that column. Follow the layers instead and say so.
    const SlicingParameters &sp = object.slicing_parameters();
    if (std::abs(sp.belt_floor_shear_factor) > EPSILON) {
        const double slope_u = sp.belt_floor_from_axis == 0 ? lean.slope.x() : lean.slope.y();
        if (1. - sp.belt_floor_shear_factor * slope_u <= 0.05) {
            lean.floor_unreachable = true;
            lean.clamped           = false;
            lean.slope             = Vec2d(0., 0.);
        }
    }
    if (! lean.active())
        // Coincides with the layer normal: follow the layers. `clamped` stays, so a limit of 0 warns.
        lean.slope = Vec2d(0., 0.);
    return lean;
}

ScopedSupportShear::ScopedSupportShear(PrintObject &object)
    : m_object(object), m_lean(support_lean(object))
{
    if (! m_lean.active())
        return;
    m_object.m_support_lean = m_lean;
    m_object.m_support_lean_shifts.reserve(m_object.m_layers.size());
    for (Layer *layer : m_object.m_layers) {
        const Point shift = m_lean.shift_at(layer->print_z);
        m_object.m_support_lean_shifts.emplace_back(shift);
        layer->translate(shift);
    }
}

ScopedSupportShear::~ScopedSupportShear()
{
    if (! m_lean.active())
        return;
    // Undo exactly the shifts the constructor applied, whatever print_z the layers hold now.
    for (size_t i = 0; i < m_object.m_layers.size() && i < m_object.m_support_lean_shifts.size(); ++ i)
        m_object.m_layers[i]->translate(- m_object.m_support_lean_shifts[i]);
    // Everything the generator built lives in the sheared frame; its own print_z (global by now,
    // see PrintObject::_generate_support_material()) says exactly how far it moved.
    for (SupportLayer *layer : m_object.m_support_layers)
        layer->translate(- m_lean.shift_at(layer->print_z));
    m_object.m_support_lean_shifts.clear();
    m_object.m_support_lean = SupportLean();
}

} // namespace Slic3r
