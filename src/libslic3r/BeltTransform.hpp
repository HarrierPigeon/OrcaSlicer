#pragma once

#include "libslic3r.h"
#include "Point.hpp"
#include "BoundingBox.hpp"
#include "PrintConfig.hpp"
#include "Geometry.hpp"

#include <cmath>

namespace Slic3r {

class ModelObject;

// Shared belt-printer transform math.
//
// The pre-slice pipeline applied in PrintObjectSlice.cpp is:
//     trafo_out = z_shift * scale * shear * pre_remap * trafo_in
//
// This class provides the building blocks so every call site uses the
// same implementation.  z_shift is object-dependent (computed from mesh
// vertex bounds) and is NOT included in build_forward_transform().
class BeltTransformPipeline
{
public:
    // ---- Pure math helpers ------------------------------------------------

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

    // ---- Identity checks --------------------------------------------------

    static bool has_preslice_remap(const PrintConfig &config)
    {
        return int(config.belt_preslice_remap_x.value) != int(BeltRemapAxis::PosX) ||
               int(config.belt_preslice_remap_y.value) != int(BeltRemapAxis::PosY) ||
               int(config.belt_preslice_remap_z.value) != int(BeltRemapAxis::PosZ);
    }

    // Overload accepting DynamicPrintConfig (used in static slicing_parameters).
    static bool has_preslice_remap(const DynamicPrintConfig &config)
    {
        auto get_int = [&](const char *key) -> int {
            auto *opt = config.option<ConfigOptionEnum<BeltRemapAxis>>(key);
            return opt ? int(opt->value) : 0;
        };
        return get_int("belt_preslice_remap_x") != int(BeltRemapAxis::PosX) ||
               get_int("belt_preslice_remap_y") != int(BeltRemapAxis::PosY) ||
               get_int("belt_preslice_remap_z") != int(BeltRemapAxis::PosZ);
    }

    static bool has_shear(const PrintConfig &config)
    {
        return config.belt_shear_x.value != BeltShearMode::None ||
               config.belt_shear_y.value != BeltShearMode::None ||
               config.belt_shear_z.value != BeltShearMode::None;
    }

    static bool has_scale(const PrintConfig &config)
    {
        double sx = compute_scale_factor(config.belt_scale_x.value, config.belt_scale_x_angle.value);
        double sy = compute_scale_factor(config.belt_scale_y.value, config.belt_scale_y_angle.value);
        double sz = compute_scale_factor(config.belt_scale_z.value, config.belt_scale_z_angle.value);
        return std::abs(sx - 1.) > EPSILON ||
               std::abs(sy - 1.) > EPSILON ||
               std::abs(sz - 1.) > EPSILON;
    }

    // ---- Matrix builders --------------------------------------------------

    // Build the pre-slice axis remap transform (includes Rev-mode translation).
    static Transform3d build_preslice_remap(const PrintConfig &config);

    // Build the 3x3 shear matrix.  Returns Identity if no shear is active.
    // Also sets has_shear_out if non-null.
    static Matrix3d build_shear_matrix(const PrintConfig &config, bool *has_shear_out = nullptr);

    // Build the 3x3 diagonal scale matrix.  Returns Identity if no scale.
    // Also sets has_scale_out if non-null.
    static Matrix3d build_scale_matrix(const PrintConfig &config, bool *has_scale_out = nullptr);

    // Combined forward transform: scale * shear * pre_remap.
    // Does NOT include the per-object Z-shift.
    static Transform3d build_forward_transform(const PrintConfig &config);

    // ---- Bounding box remap -----------------------------------------------

    // Remap a bounding box through the pre-slice axis remap.
    // Returns the original bbox if remap is identity.
    static BoundingBoxf3 remap_bbox(const BoundingBoxf3 &bb, const PrintConfig &config);
    static BoundingBoxf3 remap_bbox(const ModelObject &model_object, const PrintConfig &config);

    // ---- Belt floor parameters --------------------------------------------

    struct BeltFloorParams {
        double shear_factor = 0.0;
        int    from_axis    = 1;
        double z_shift      = 0.0;
    };

    // Result of computing belt height + floor params.
    struct BeltHeightResult {
        double          object_height;  // Effective object height after shear/scale
        BeltFloorParams floor_params;
    };

    // Compute effective object height and belt floor parameters from config
    // and pre-remapped bounding box.  original_height is the input height
    // (bb.size().z() or model_object.max_z()).
    static BeltHeightResult compute_belt_height_and_floor(
        const PrintConfig &config, const BoundingBoxf3 &remapped_bbox,
        double original_height);

    // Overload for DynamicPrintConfig (used by static slicing_parameters).
    static BeltHeightResult compute_belt_height_and_floor(
        const DynamicPrintConfig &config, const BoundingBoxf3 &remapped_bbox,
        double original_height);
};

} // namespace Slic3r
