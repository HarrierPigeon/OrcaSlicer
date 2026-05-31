#include "MachineFrameTransform.hpp"
#include "../BeltTransform.hpp"
#include "../BoundingBox.hpp"

namespace Slic3r {

namespace {

// Build the 3x3 shear matrix from gcode_shear_* keys.
Matrix3d build_gcode_shear_matrix(const PrintConfig &config, bool &active)
{
    struct AxisShear { BeltShearMode mode; double angle; int from; };
    AxisShear axes[3] = {
        { config.gcode_shear_x.value, config.gcode_shear_x_angle.value, int(config.gcode_shear_x_from.value) },
        { config.gcode_shear_y.value, config.gcode_shear_y_angle.value, int(config.gcode_shear_y_from.value) },
        { config.gcode_shear_z.value, config.gcode_shear_z_angle.value, int(config.gcode_shear_z_from.value) },
    };

    Matrix3d shear = Matrix3d::Identity();
    active = false;
    for (int row = 0; row < 3; ++row) {
        if (axes[row].mode != BeltShearMode::None) {
            double factor = BeltTransformPipeline::compute_shear_factor(axes[row].mode, axes[row].angle);
            if (std::abs(factor) > EPSILON) {
                shear(row, axes[row].from) += factor;
                active = true;
            }
        }
    }
    return shear;
}

// Build the 3x3 diagonal scale matrix from gcode_scale_* keys.
Matrix3d build_gcode_scale_matrix(const PrintConfig &config, bool &active)
{
    double sx = BeltTransformPipeline::compute_scale_factor(config.gcode_scale_x.value, config.gcode_scale_x_angle.value);
    double sy = BeltTransformPipeline::compute_scale_factor(config.gcode_scale_y.value, config.gcode_scale_y_angle.value);
    double sz = BeltTransformPipeline::compute_scale_factor(config.gcode_scale_z.value, config.gcode_scale_z_angle.value);

    active = (std::abs(sx - 1.) > EPSILON ||
              std::abs(sy - 1.) > EPSILON ||
              std::abs(sz - 1.) > EPSILON);

    Matrix3d scale = Matrix3d::Identity();
    if (active) {
        scale(0, 0) = sx;
        scale(1, 1) = sy;
        scale(2, 2) = sz;
    }
    return scale;
}

} // namespace

bool MachineFrameTransform::init_from_config(const PrintConfig &config)
{
    m_active    = false;
    m_transform = Transform3d::Identity();

    if (!config.belt_printer.value)
        return false;

    bool        shear_active = false;
    Matrix3d    shear        = build_gcode_shear_matrix(config, shear_active);
    bool        scale_active = false;
    Matrix3d    scale        = build_gcode_scale_matrix(config, scale_active);

    if (!shear_active && !scale_active)
        return false;

    // Compose per belt_gcode_transform_order:
    //   ScaleThenShear: applied to p, scale runs first then shear (shear * scale).
    //   ShearThenScale: applied to p, shear runs first then scale (scale * shear).
    Transform3d combined = Transform3d::Identity();
    combined.linear() = (config.belt_gcode_transform_order.value == BeltTransformOrder::ScaleThenShear)
        ? Matrix3d(shear * scale)
        : Matrix3d(scale * shear);

    if (combined.isApprox(Transform3d::Identity()))
        return false;

    m_transform = combined;
    m_active    = true;
    return true;
}

Vec3d MachineFrameTransform::apply(const Vec3d &pos) const
{
    if (!m_active)
        return pos;
    return m_transform * pos;
}

} // namespace Slic3r
