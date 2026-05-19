#include "MachineFrameTransform.hpp"
#include "../BeltTransform.hpp"
#include "../BoundingBox.hpp"

namespace Slic3r {

namespace {

// Build the post-gcode axis-remap transform (mirrors BeltTransformPipeline::build_preslice_remap
// but reads post_gcode_remap_* keys).  Includes Rev-mode translation derived from build volume.
Transform3d build_post_gcode_remap(const PrintConfig &config)
{
    Transform3d remap = Transform3d::Identity();

    int rx = int(config.post_gcode_remap_x.value);
    int ry = int(config.post_gcode_remap_y.value);
    int rz = int(config.post_gcode_remap_z.value);

    if (rx == int(RemapAxis::PosX) && ry == int(RemapAxis::PosY) && rz == int(RemapAxis::PosZ))
        return remap;

    auto remap_column = [](int r) -> Vec3d {
        int axis = r % 3;
        Vec3d col = Vec3d::Zero();
        if (r < 3)      col[axis] =  1.0;
        else if (r < 6) col[axis] = -1.0;
        else            col[axis] = -1.0;  // Rev: max - pos
        return col;
    };

    Matrix3d lin;
    lin.col(0) = remap_column(rx);
    lin.col(1) = remap_column(ry);
    lin.col(2) = remap_column(rz);
    remap.linear() = lin;

    if (rx >= 6 || ry >= 6 || rz >= 6) {
        BoundingBoxf bbox_bed(config.printable_area.values);
        Vec3d vol_max(bbox_bed.max.x(), bbox_bed.max.y(), config.printable_height.value);
        Vec3d trans = Vec3d::Zero();
        auto add_rev = [&](int r, int out) {
            if (r >= 6) trans[out] = vol_max[r % 3];
        };
        add_rev(rx, 0);
        add_rev(ry, 1);
        add_rev(rz, 2);
        remap.translation() = trans;
    }

    return remap;
}

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

bool has_post_gcode_remap(const PrintConfig &config)
{
    return int(config.post_gcode_remap_x.value) != int(RemapAxis::PosX) ||
           int(config.post_gcode_remap_y.value) != int(RemapAxis::PosY) ||
           int(config.post_gcode_remap_z.value) != int(RemapAxis::PosZ);
}

} // namespace

bool MachineFrameTransform::init_from_config(const PrintConfig &config)
{
    m_active    = false;
    m_transform = Transform3d::Identity();

    if (!config.belt_printer.value)
        return false;

    Transform3d post_remap   = build_post_gcode_remap(config);
    bool        shear_active = false;
    Matrix3d    shear        = build_gcode_shear_matrix(config, shear_active);
    bool        scale_active = false;
    Matrix3d    scale        = build_gcode_scale_matrix(config, scale_active);

    if (!shear_active && !scale_active && !has_post_gcode_remap(config))
        return false;

    // Compose per belt_gcode_transform_order:
    //   ScaleThenShear: applied to p, scale runs first then shear (shear * scale).
    //   ShearThenScale: applied to p, shear runs first then scale (scale * shear).
    Transform3d combined = Transform3d::Identity();
    combined.linear() = (config.belt_gcode_transform_order.value == BeltTransformOrder::ScaleThenShear)
        ? Matrix3d(shear * scale)
        : Matrix3d(scale * shear);
    combined = combined * post_remap;

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
