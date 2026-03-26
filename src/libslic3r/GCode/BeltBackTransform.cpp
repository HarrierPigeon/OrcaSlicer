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
    m_inverse = Matrix3d::Identity();

    if (!config.belt_printer.value || !config.belt_gcode_back_transform.value)
        return false;

    // Require at least one shear axis with global mode enabled.
    if (!config.belt_shear_x_global.value &&
        !config.belt_shear_y_global.value &&
        !config.belt_shear_z_global.value)
        return false;

    // Build per-axis shear matrix (same as PrintObjectSlice.cpp lines 160-177).
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

    // Build per-axis scale diagonal matrix (same as PrintObjectSlice.cpp lines 194-204).
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

    if (!has_shear && !has_scale)
        return false;

    // combined = scale * shear  (same order as PrintObjectSlice.cpp line 208).
    Matrix3d combined = scale * shear;
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
