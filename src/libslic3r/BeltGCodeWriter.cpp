#include "BeltGCodeWriter.hpp"
#include "FirstLayerPlane.hpp"
#include "Geometry.hpp"

namespace Slic3r {

namespace {

// Decide whether a particular destination point gets first-layer treatment.
// When the plane evaluator is active, distance from the plane wins; otherwise
// fall back to the layer-coarse m_is_first_layer flag set by the caller.
inline bool belt_point_on_first_layer(
    const FirstLayerPlane *plane,
    double                 first_layer_thickness_mm,
    bool                   layer_first_flag,
    const Vec3d           &point_slicing_mm)
{
    if (plane && plane->is_active())
        return plane->is_first_layer(point_slicing_mm, first_layer_thickness_mm);
    return layer_first_flag;
}

} // namespace

// ---- Belt configuration ---------------------------------------------------

void BeltGCodeWriter::set_belt_angle(double angle_deg)
{
    m_belt_angle_rad = Geometry::deg2rad(angle_deg);
}

void BeltGCodeWriter::set_belt_back_transform(const PrintConfig &config)
{
    m_belt_back_transform.init_from_config(config);
}

void BeltGCodeWriter::set_origin_snap(int axis, bool enable, double offset, double bbox_min)
{
    if (axis >= 0 && axis < 3) {
        m_origin_snap[axis]     = enable;
        m_origin_offset[axis]   = offset;
        m_origin_bbox_min[axis] = bbox_min;
    }
}

Vec3d BeltGCodeWriter::to_machine_coords(const Vec3d &pos) const
{
    // Step 1: Undo the shear/scale applied during slicing.
    Vec3d p = m_belt_back_transform.apply(pos);
    // Step 2: Apply axis remap (uses inherited base class method).
    Vec3d result = apply_axis_remap(p);
    // Step 3: Per-axis origin snap.
    for (int i = 0; i < 3; ++i)
        if (m_origin_snap[i])
            result[i] -= (m_origin_bbox_min[i] - m_origin_offset[i]);
    return result;
}

// ---- Overridden movement methods ------------------------------------------

std::string BeltGCodeWriter::travel_to_xy(const Vec2d &point, const std::string &comment)
{
    m_pos(0) = point(0);
    m_pos(1) = point(1);

    this->set_current_position_clear(true);
    Vec2d point_on_plate = { point(0) - m_x_offset, point(1) - m_y_offset };

    // Belt printer: transform to machine coordinates (XY travel also needs Z due to YZ rotation)
    Vec3d machine = to_machine_coords(Vec3d(point_on_plate.x(), point_on_plate.y(), m_pos.z()));

    GCodeG1Formatter w;
    w.emit_xyz(machine);
    const bool first_layer_for_point = belt_point_on_first_layer(
        m_first_layer_plane, m_first_layer_thickness_mm, m_is_first_layer,
        Vec3d(point.x(), point.y(), m_pos.z()));
    auto speed = first_layer_for_point
        ? this->config.get_abs_value("initial_layer_travel_speed") : this->config.travel_speed.value;
    w.emit_f(speed * 60.0);
    w.emit_comment(GCodeWriter::full_gcode_comment, comment);
    return w.string();
}

std::string BeltGCodeWriter::lazy_lift(LiftType lift_type, bool spiral_vase)
{
    // Belt printer: force NormalLift since SpiralLift and SlopeLift compute
    // slope angles that don't account for the YZ coordinate rotation.
    return GCodeWriter::lazy_lift(LiftType::NormalLift, spiral_vase);
}

std::string BeltGCodeWriter::eager_lift(const LiftType type)
{
    // Belt printer: force NormalLift (SpiralLift/SlopeLift don't account for YZ rotation).
    return GCodeWriter::eager_lift(LiftType::NormalLift);
}

std::string BeltGCodeWriter::_travel_to_z(double z, const std::string &comment)
{
    m_pos(2) = z;

    double speed = this->config.travel_speed_z.value;
    if (speed == 0.) {
        const bool first_layer_for_point = belt_point_on_first_layer(
            m_first_layer_plane, m_first_layer_thickness_mm, m_is_first_layer,
            Vec3d(m_pos.x(), m_pos.y(), z));
        speed = first_layer_for_point ? this->config.get_abs_value("initial_layer_travel_speed")
                                      : this->config.travel_speed.value;
    }

    // Belt printer: a Z-only move in slicing frame needs to emit both Y and Z in machine coords.
    Vec3d machine = to_machine_coords(Vec3d(m_pos.x() - m_x_offset, m_pos.y() - m_y_offset, z));

    GCodeG1Formatter w;
    w.emit_xyz(machine);
    w.emit_f(speed * 60.0);
    w.emit_comment(GCodeWriter::full_gcode_comment, comment);
    return w.string();
}

std::string BeltGCodeWriter::extrude_to_xy(const Vec2d &point, double dE, const std::string &comment, bool force_no_extrusion)
{
    m_pos(0) = point(0);
    m_pos(1) = point(1);
    if (std::abs(dE) <= std::numeric_limits<double>::epsilon())
        force_no_extrusion = true;

    if (!force_no_extrusion)
        filament()->extrude(dE);

    Vec2d point_on_plate = { point(0) - m_x_offset, point(1) - m_y_offset };

    // Belt printer: transform and emit XYZ (Y and Z are coupled)
    Vec3d machine = to_machine_coords(Vec3d(point_on_plate.x(), point_on_plate.y(), m_pos.z()));

    GCodeG1Formatter w;
    w.emit_xyz(machine);
    if (!force_no_extrusion)
        w.emit_e(filament()->E());
    w.emit_comment(GCodeWriter::full_gcode_comment, comment);
    return w.string();
}

std::string BeltGCodeWriter::extrude_to_xyz(const Vec3d &point, double dE, const std::string &comment, bool force_no_extrusion)
{
    m_pos = point;
    m_lifted = 0;
    if (!force_no_extrusion)
        filament()->extrude(dE);

    Vec3d point_on_plate = { point(0) - m_x_offset, point(1) - m_y_offset, point(2) };
    point_on_plate = to_machine_coords(point_on_plate);

    GCodeG1Formatter w;
    w.emit_xyz(point_on_plate);
    if (!force_no_extrusion)
        w.emit_e(filament()->E());
    w.emit_comment(GCodeWriter::full_gcode_comment, comment);
    return w.string();
}

std::string BeltGCodeWriter::travel_to_xyz(const Vec3d &point, const std::string &comment, bool force_z)
{
    // Belt-specific override of travel_to_xyz.
    // Key differences from base:
    // 1. All coordinates go through to_machine_coords()
    // 2. Always emit full XYZ (can't split XY and Z due to coupling)
    // 3. Lift type forced to NormalLift (handled by lazy_lift/eager_lift overrides)

    Vec3d dest_point = point;
    const bool first_layer_for_point = belt_point_on_first_layer(
        m_first_layer_plane, m_first_layer_thickness_mm, m_is_first_layer, point);
    auto travel_speed =
        first_layer_for_point ? this->config.get_abs_value("initial_layer_travel_speed")
                              : this->config.travel_speed.value;

    // Handle pending z_hop
    if (std::abs(m_to_lift) > EPSILON) {
        assert(std::abs(m_lifted) < EPSILON);
        if ((!this->is_current_position_clear() || m_pos != dest_point) &&
            m_to_lift + m_pos(2) > point(2)) {
            m_lifted = m_to_lift + m_pos(2) - point(2);
            dest_point(2) = m_to_lift + m_pos(2);
        }
        m_to_lift = 0.;

        std::string slop_move;
        Vec3d source = { m_pos(0) - m_x_offset, m_pos(1) - m_y_offset, m_pos(2) };
        Vec3d target = { dest_point(0) - m_x_offset, dest_point(1) - m_y_offset, dest_point(2) };
        Vec3d delta = target - source;
        Vec2d delta_no_z = { delta(0), delta(1) };

        if (delta(2) > 0 && delta_no_z.norm() != 0.0f) {
            // Belt: SpiralLift and SlopeLift are disabled (lazy_lift forces NormalLift),
            // but handle NormalLift and fallthrough.
            if (m_to_lift_type == LiftType::SlopeLift &&
                this->is_current_position_clear() &&
                atan2(delta(2), delta_no_z.norm()) < this->filament()->travel_slope()) {
                Vec2d temp = delta_no_z.normalized() * delta(2) / tan(this->filament()->travel_slope());
                Vec3d slope_top_point = Vec3d(temp(0), temp(1), delta(2)) + source;
                slope_top_point = to_machine_coords(slope_top_point);
                GCodeG1Formatter w0;
                w0.emit_xyz(slope_top_point);
                w0.emit_f(travel_speed * 60.0);
                w0.emit_comment(GCodeWriter::full_gcode_comment, comment);
                slop_move = w0.string();
            }
            else if (m_to_lift_type == LiftType::NormalLift) {
                slop_move = _travel_to_z(target.z(), "normal lift Z");
            }
        }

        std::string xy_z_move;
        {
            Vec3d emit_target = to_machine_coords(target);
            GCodeG1Formatter w0;
            // Belt mode: always emit full XYZ since Y and Z are coupled
            w0.emit_xyz(emit_target);
            w0.emit_f(travel_speed * 60.0);
            w0.emit_comment(GCodeWriter::full_gcode_comment, comment);
            xy_z_move = w0.string();
        }
        m_pos = dest_point;
        this->set_current_position_clear(true);
        return slop_move + xy_z_move;
    }
    else if (!force_z && !this->will_move_z(point(2))) {
        double nominal_z = m_pos(2) - m_lifted;
        m_lifted -= (point(2) - nominal_z);
        if (std::abs(m_lifted) < EPSILON)
            m_lifted = 0.;
        this->set_current_position_clear(true);
        return this->travel_to_xy(to_2d(point));
    }
    else {
        m_lifted = 0;
    }

    Vec3d point_on_plate = { dest_point(0) - m_x_offset, dest_point(1) - m_y_offset, dest_point(2) };
    point_on_plate = to_machine_coords(point_on_plate);

    // Belt mode: always emit full XYZ
    GCodeG1Formatter w;
    w.emit_xyz(point_on_plate);
    w.emit_f(this->config.travel_speed.value * 60.0);
    w.emit_comment(GCodeWriter::full_gcode_comment, comment);

    m_pos = dest_point;
    this->set_current_position_clear(true);
    return w.string();
}

} // namespace Slic3r
