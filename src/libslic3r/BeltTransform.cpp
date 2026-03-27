#include "BeltTransform.hpp"
#include "Model.hpp"

#include <limits>

namespace Slic3r {

// ---- Matrix builders ------------------------------------------------------

Transform3d BeltTransformPipeline::build_preslice_remap(const PrintConfig &config)
{
    Transform3d pre_remap = Transform3d::Identity();
    if (!has_preslice_remap(config))
        return pre_remap;

    int pre_rx = int(config.belt_preslice_remap_x.value);
    int pre_ry = int(config.belt_preslice_remap_y.value);
    int pre_rz = int(config.belt_preslice_remap_z.value);

    // Each remap value selects a source axis and sign.
    auto remap_column = [](int r) -> Vec3d {
        int axis = r % 3;
        Vec3d col = Vec3d::Zero();
        if (r < 3)      col[axis] =  1.0;  // +axis
        else if (r < 6) col[axis] = -1.0;  // -axis
        else            col[axis] = -1.0;  // Rev: max - pos = -(pos - max)
        return col;
    };

    Matrix3d remap_lin;
    remap_lin.col(0) = remap_column(pre_rx);
    remap_lin.col(1) = remap_column(pre_ry);
    remap_lin.col(2) = remap_column(pre_rz);
    pre_remap.linear() = remap_lin;

    // Translation for Rev modes (needs build volume extents).
    if (pre_rx >= 6 || pre_ry >= 6 || pre_rz >= 6) {
        BoundingBoxf bbox_bed(config.printable_area.values);
        Vec3d vol_max(bbox_bed.max.x(), bbox_bed.max.y(),
                      config.printable_height.value);
        Vec3d remap_trans = Vec3d::Zero();
        auto add_rev = [&](int r, int out) {
            if (r >= 6) remap_trans[out] = vol_max[r % 3];
        };
        add_rev(pre_rx, 0);
        add_rev(pre_ry, 1);
        add_rev(pre_rz, 2);
        pre_remap.translation() = remap_trans;
    }

    return pre_remap;
}

Matrix3d BeltTransformPipeline::build_shear_matrix(const PrintConfig &config, bool *has_shear_out)
{
    struct AxisShear { BeltShearMode mode; double angle; int from; };
    AxisShear axes[3] = {
        { config.belt_shear_x.value, config.belt_shear_x_angle.value, int(config.belt_shear_x_from.value) },
        { config.belt_shear_y.value, config.belt_shear_y_angle.value, int(config.belt_shear_y_from.value) },
        { config.belt_shear_z.value, config.belt_shear_z_angle.value, int(config.belt_shear_z_from.value) },
    };

    Matrix3d shear = Matrix3d::Identity();
    bool active = false;
    for (int row = 0; row < 3; ++row) {
        if (axes[row].mode != BeltShearMode::None) {
            double factor = compute_shear_factor(axes[row].mode, axes[row].angle);
            if (std::abs(factor) > EPSILON) {
                shear(row, axes[row].from) += factor;
                active = true;
            }
        }
    }
    if (has_shear_out) *has_shear_out = active;
    return shear;
}

Matrix3d BeltTransformPipeline::build_scale_matrix(const PrintConfig &config, bool *has_scale_out)
{
    double sx = compute_scale_factor(config.belt_scale_x.value, config.belt_scale_x_angle.value);
    double sy = compute_scale_factor(config.belt_scale_y.value, config.belt_scale_y_angle.value);
    double sz = compute_scale_factor(config.belt_scale_z.value, config.belt_scale_z_angle.value);

    bool active = (std::abs(sx - 1.) > EPSILON ||
                   std::abs(sy - 1.) > EPSILON ||
                   std::abs(sz - 1.) > EPSILON);

    Matrix3d scale = Matrix3d::Identity();
    if (active) {
        scale(0, 0) = sx;
        scale(1, 1) = sy;
        scale(2, 2) = sz;
    }
    if (has_scale_out) *has_scale_out = active;
    return scale;
}

Transform3d BeltTransformPipeline::build_forward_transform(const PrintConfig &config)
{
    Transform3d pre_remap   = build_preslice_remap(config);
    bool        shear_active = false;
    Matrix3d    shear        = build_shear_matrix(config, &shear_active);
    bool        scale_active = false;
    Matrix3d    scale        = build_scale_matrix(config, &scale_active);

    // Pipeline: scale * shear * pre_remap
    Transform3d combined = Transform3d::Identity();
    combined.linear() = scale * shear;
    combined = combined * pre_remap;
    return combined;
}

// ---- Bounding box remap ---------------------------------------------------

BoundingBoxf3 BeltTransformPipeline::remap_bbox(const BoundingBoxf3 &bb, const PrintConfig &config)
{
    int pre_rx = int(config.belt_preslice_remap_x.value);
    int pre_ry = int(config.belt_preslice_remap_y.value);
    int pre_rz = int(config.belt_preslice_remap_z.value);

    if (pre_rx == int(BeltRemapAxis::PosX) &&
        pre_ry == int(BeltRemapAxis::PosY) &&
        pre_rz == int(BeltRemapAxis::PosZ))
        return bb;  // Identity remap.

    auto remap_coord = [](int r, const Vec3d &v) -> double {
        int axis = r % 3;
        if (r < 3) return v[axis];
        return -v[axis];
    };

    Vec3d mn = bb.min.cast<double>(), mx = bb.max.cast<double>();
    BoundingBoxf3 rbb;
    for (int i = 0; i < 8; ++i) {
        Vec3d c((i & 1) ? mx.x() : mn.x(),
                (i & 2) ? mx.y() : mn.y(),
                (i & 4) ? mx.z() : mn.z());
        Vec3d rc(remap_coord(pre_rx, c), remap_coord(pre_ry, c), remap_coord(pre_rz, c));
        if (i == 0) rbb = BoundingBoxf3(rc, rc);
        else rbb.merge(rc);
    }
    return rbb;
}

BoundingBoxf3 BeltTransformPipeline::remap_bbox(const ModelObject &model_object, const PrintConfig &config)
{
    return remap_bbox(model_object.raw_bounding_box(), config);
}

// ---- Belt floor parameters ------------------------------------------------

// Shared implementation for both PrintConfig and DynamicPrintConfig.
// Template avoids duplicating the math for the two config types.
namespace {

template<typename Config>
BeltTransformPipeline::BeltHeightResult compute_belt_height_and_floor_impl(
    const Config &config, const BoundingBoxf3 &bb, double original_height)
{
    BeltTransformPipeline::BeltHeightResult result;
    result.object_height = original_height;

    // Extract Z-axis shear/scale config.
    BeltShearMode z_shear_mode;
    double        z_shear_angle;
    BeltScaleMode z_scale_mode;
    double        z_scale_angle;
    int           z_shear_from;

    if constexpr (std::is_same_v<Config, PrintConfig>) {
        z_shear_mode  = config.belt_shear_z.value;
        z_shear_angle = config.belt_shear_z_angle.value;
        z_scale_mode  = config.belt_scale_z.value;
        z_scale_angle = config.belt_scale_z_angle.value;
        z_shear_from  = int(config.belt_shear_z_from.value);
    } else {
        // DynamicPrintConfig path
        auto get_shear = [&](const char *key) {
            auto *opt = config.template option<ConfigOptionEnum<BeltShearMode>>(key);
            return opt ? opt->value : BeltShearMode::None;
        };
        auto get_scale = [&](const char *key) {
            auto *opt = config.template option<ConfigOptionEnum<BeltScaleMode>>(key);
            return opt ? opt->value : BeltScaleMode::None;
        };
        auto get_float = [&](const char *key) {
            auto *opt = config.template option<ConfigOptionFloat>(key);
            return opt ? opt->value : 45.0;
        };
        auto get_axis = [&](const char *key) {
            auto *opt = config.template option<ConfigOptionEnum<BeltAxis>>(key);
            return opt ? int(opt->value) : 1;
        };
        z_shear_mode  = get_shear("belt_shear_z");
        z_shear_angle = get_float("belt_shear_z_angle");
        z_scale_mode  = get_scale("belt_scale_z");
        z_scale_angle = get_float("belt_scale_z_angle");
        z_shear_from  = get_axis("belt_shear_z_from");
    }

    bool has_z_shear = z_shear_mode != BeltShearMode::None;
    bool has_z_scale = z_scale_mode != BeltScaleMode::None;

    if (!has_z_shear && !has_z_scale)
        return result;

    double shear_factor = has_z_shear
        ? BeltTransformPipeline::compute_shear_factor(z_shear_mode, z_shear_angle) : 0.;
    double scale_z = BeltTransformPipeline::compute_scale_factor(z_scale_mode, z_scale_angle);

    if (has_z_shear && std::abs(shear_factor) > EPSILON) {
        int from = z_shear_from;
        double min_rz = std::numeric_limits<double>::max();
        double max_rz = std::numeric_limits<double>::lowest();
        for (double vz : {bb.min.z(), bb.max.z()})
            for (double vs : {bb.min(from), bb.max(from)}) {
                double new_z = scale_z * (vz + shear_factor * vs);
                min_rz = std::min(min_rz, new_z);
                max_rz = std::max(max_rz, new_z);
            }
        result.object_height              = max_rz - min_rz;
        result.floor_params.shear_factor  = shear_factor;
        result.floor_params.from_axis     = from;
        result.floor_params.z_shift       = bb.min.z() + ((min_rz < 0.) ? -min_rz : 0.);
    } else {
        result.object_height = original_height * scale_z;
    }

    return result;
}

} // anonymous namespace

BeltTransformPipeline::BeltHeightResult BeltTransformPipeline::compute_belt_height_and_floor(
    const PrintConfig &config, const BoundingBoxf3 &remapped_bbox, double original_height)
{
    return compute_belt_height_and_floor_impl(config, remapped_bbox, original_height);
}

BeltTransformPipeline::BeltHeightResult BeltTransformPipeline::compute_belt_height_and_floor(
    const DynamicPrintConfig &config, const BoundingBoxf3 &remapped_bbox, double original_height)
{
    return compute_belt_height_and_floor_impl(config, remapped_bbox, original_height);
}

} // namespace Slic3r
