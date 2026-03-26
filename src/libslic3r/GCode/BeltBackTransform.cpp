#include "BeltBackTransform.hpp"
#include "../Geometry.hpp"
#include <cmath>

namespace Slic3r {

// Keep in sync with PrintObjectSlice.cpp compute_shear_factor (lines ~147-157).
static double compute_shear_factor(BeltShearMode mode, double angle_deg)
{
    double angle_rad = Geometry::deg2rad(angle_deg);
    double sin_a     = std::sin(angle_rad);
    double cos_a     = std::cos(angle_rad);
    switch (mode) {
    case BeltShearMode::PosCot: return (sin_a > EPSILON) ?  cos_a / sin_a : 0.;
    case BeltShearMode::NegCot: return (sin_a > EPSILON) ? -cos_a / sin_a : 0.;
    case BeltShearMode::PosTan: return (cos_a > EPSILON) ?  sin_a / cos_a : 0.;
    case BeltShearMode::NegTan: return (cos_a > EPSILON) ? -sin_a / cos_a : 0.;
    default: return 0.;
    }
}

// Keep in sync with PrintObjectSlice.cpp compute_scale_factor (lines ~180-192).
static double compute_scale_factor(BeltScaleMode mode, double angle_deg)
{
    if (mode == BeltScaleMode::None) return 1.;
    double angle_rad = Geometry::deg2rad(angle_deg);
    double sin_a     = std::sin(angle_rad);
    double cos_a     = std::cos(angle_rad);
    switch (mode) {
    case BeltScaleMode::InvSin: return (sin_a > EPSILON) ? 1. / sin_a : 1.;
    case BeltScaleMode::InvCos: return (cos_a > EPSILON) ? 1. / cos_a : 1.;
    case BeltScaleMode::Sin:    return sin_a;
    case BeltScaleMode::Cos:    return cos_a;
    default: return 1.;
    }
}

bool BeltBackTransform::init_from_config(const PrintConfig &config)
{
    m_active  = false;
    m_inverse = Transform3d::Identity();

    if (!config.belt_printer.value || !config.belt_gcode_back_transform.value)
        return false;

    // --- Pre-slice axis remap (same as PrintObjectSlice.cpp) ---
    int pre_rx = int(config.belt_preslice_remap_x.value);
    int pre_ry = int(config.belt_preslice_remap_y.value);
    int pre_rz = int(config.belt_preslice_remap_z.value);

    bool has_preslice_remap = (pre_rx != int(BeltRemapAxis::PosX) ||
                               pre_ry != int(BeltRemapAxis::PosY) ||
                               pre_rz != int(BeltRemapAxis::PosZ));

    // Require at least one active transform to proceed.
    bool has_global_shear = config.belt_shear_x_global.value ||
                            config.belt_shear_y_global.value ||
                            config.belt_shear_z_global.value;
    if (!has_global_shear && !has_preslice_remap)
        return false;

    // Build pre-slice remap matrix.
    Transform3d pre_remap = Transform3d::Identity();
    if (has_preslice_remap) {
        auto remap_column = [](int r) -> Vec3d {
            int axis = r % 3;
            Vec3d col = Vec3d::Zero();
            if (r < 3)      col[axis] =  1.0;
            else if (r < 6) col[axis] = -1.0;
            else            col[axis] = -1.0;  // Rev: max - pos
            return col;
        };

        Matrix3d remap_lin;
        remap_lin.col(0) = remap_column(pre_rx);
        remap_lin.col(1) = remap_column(pre_ry);
        remap_lin.col(2) = remap_column(pre_rz);
        pre_remap.linear() = remap_lin;

        // Rev mode translation (needs build volume extents).
        Vec3d remap_trans = Vec3d::Zero();
        if (pre_rx >= 6 || pre_ry >= 6 || pre_rz >= 6) {
            BoundingBoxf bbox_bed(config.printable_area.values);
            Vec3d vol_max(bbox_bed.max.x(), bbox_bed.max.y(),
                          config.printable_height.value);
            auto add_rev = [&](int r, int out) {
                if (r >= 6) remap_trans[out] = vol_max[r % 3];
            };
            add_rev(pre_rx, 0);
            add_rev(pre_ry, 1);
            add_rev(pre_rz, 2);
        }
        pre_remap.translation() = remap_trans;
    }

    // Build per-axis shear matrix (same as PrintObjectSlice.cpp).
    struct AxisShear { BeltShearMode mode; double angle; int from; };
    AxisShear axes[3] = {
        { config.belt_shear_x.value, config.belt_shear_x_angle.value, int(config.belt_shear_x_from.value) },
        { config.belt_shear_y.value, config.belt_shear_y_angle.value, int(config.belt_shear_y_from.value) },
        { config.belt_shear_z.value, config.belt_shear_z_angle.value, int(config.belt_shear_z_from.value) },
    };

    Matrix3d shear = Matrix3d::Identity();
    bool has_shear = false;
    for (int row = 0; row < 3; ++row) {
        if (axes[row].mode != BeltShearMode::None) {
            double factor = compute_shear_factor(axes[row].mode, axes[row].angle);
            if (std::abs(factor) > EPSILON) {
                shear(row, axes[row].from) += factor;
                has_shear = true;
            }
        }
    }

    // Build per-axis scale diagonal matrix (same as PrintObjectSlice.cpp).
    double sx = compute_scale_factor(config.belt_scale_x.value, config.belt_scale_x_angle.value);
    double sy = compute_scale_factor(config.belt_scale_y.value, config.belt_scale_y_angle.value);
    double sz = compute_scale_factor(config.belt_scale_z.value, config.belt_scale_z_angle.value);

    Matrix3d scale = Matrix3d::Identity();
    bool has_scale = (std::abs(sx - 1.) > EPSILON ||
                      std::abs(sy - 1.) > EPSILON ||
                      std::abs(sz - 1.) > EPSILON);
    if (has_scale) {
        scale(0, 0) = sx;
        scale(1, 1) = sy;
        scale(2, 2) = sz;
    }

    if (!has_shear && !has_scale && !has_preslice_remap)
        return false;

    // Forward pipeline: scale * shear * pre_remap  (same order as PrintObjectSlice.cpp).
    Transform3d combined = Transform3d::Identity();
    combined.linear() = scale * shear;
    combined = combined * pre_remap;
    m_inverse = combined.inverse();
    m_active  = true;
    return true;
}

Vec3d BeltBackTransform::apply(const Vec3d &pos) const
{
    if (!m_active)
        return pos;
    return m_inverse * pos;
}

} // namespace Slic3r
