#pragma once

#include "libslic3r.h"
#include "Point.hpp"
#include "PrintConfig.hpp"
#include <Eigen/Geometry>

namespace Slic3r {

// Utility struct that decouples slicing direction, gravity direction, and build plate normal
// into independent, configurable 3D direction vectors. Normalizes input vectors, computes
// derived quaternions/transforms, and provides convenience queries.
struct SlicingDirections {
    Vec3d slicing_dir;            // normalized, default (0,0,1)
    Vec3d gravity_dir;            // normalized, default (0,0,-1)
    Vec3d plate_normal;           // normalized, default (0,0,1)

    Eigen::Quaterniond q_slice_to_z;      // rotates slicing_dir -> +Z
    Transform3d        trafo_slice_align;  // rotation matrix form of above
    Vec3d              gravity_in_slice_frame; // gravity after slicing rotation

    bool is_default() const {
        return !has_custom_slicing() && !has_custom_gravity() && !has_custom_plate_normal();
    }

    bool has_custom_slicing() const {
        return (slicing_dir - Vec3d::UnitZ()).norm() > EPSILON;
    }

    bool has_custom_gravity() const {
        return (gravity_dir - Vec3d(0, 0, -1)).norm() > EPSILON;
    }

    bool has_custom_plate_normal() const {
        return (plate_normal - Vec3d::UnitZ()).norm() > EPSILON;
    }

    // Compute Z extent of a bounding box after slicing rotation.
    // Returns {z_min, z_max} in the rotated frame.
    std::pair<double, double> rotated_z_range(const BoundingBoxf3& bbox) const {
        double z_min = std::numeric_limits<double>::max();
        double z_max = std::numeric_limits<double>::lowest();
        for (int i = 0; i < 8; ++i) {
            Vec3d corner(
                (i & 1) ? bbox.max.x() : bbox.min.x(),
                (i & 2) ? bbox.max.y() : bbox.min.y(),
                (i & 4) ? bbox.max.z() : bbox.min.z());
            double z = (q_slice_to_z * corner).z();
            z_min = std::min(z_min, z);
            z_max = std::max(z_max, z);
        }
        return {z_min, z_max};
    }

    // Height of a bounding box after slicing rotation.
    double rotated_height(const BoundingBoxf3& bbox) const {
        if (!has_custom_slicing()) return bbox.size().z();
        auto [z_min, z_max] = rotated_z_range(bbox);
        return z_max - z_min;
    }

    // Build a complete slicing transform: rotation + Z shift so the rotated mesh starts at Z=0.
    Transform3d trafo_for_slicing(const BoundingBoxf3& bbox) const {
        if (!has_custom_slicing()) return trafo_slice_align;
        auto [z_min, z_max] = rotated_z_range(bbox);
        Transform3d t = Transform3d::Identity();
        t.pretranslate(Vec3d(0, 0, -z_min));
        return t * trafo_slice_align;
    }

    static SlicingDirections from_config(const PrintConfig& cfg) {
        return build(cfg.slicing_direction.value,
                     cfg.gravity_direction.value,
                     cfg.build_plate_normal.value);
    }

    static SlicingDirections from_config(const DynamicPrintConfig& cfg) {
        Vec3d sd(0, 0, 1), gd(0, 0, -1), pn(0, 0, 1);
        if (const auto* opt = cfg.option<ConfigOptionPoint3>("slicing_direction"))
            sd = opt->value;
        if (const auto* opt = cfg.option<ConfigOptionPoint3>("gravity_direction"))
            gd = opt->value;
        if (const auto* opt = cfg.option<ConfigOptionPoint3>("build_plate_normal"))
            pn = opt->value;
        return build(sd, gd, pn);
    }

private:
    static Vec3d safe_normalize(const Vec3d& v, const Vec3d& fallback) {
        double len = v.norm();
        return len > EPSILON ? v / len : fallback;
    }

    static SlicingDirections build(const Vec3d& sd_raw, const Vec3d& gd_raw, const Vec3d& pn_raw) {
        SlicingDirections d;
        d.slicing_dir  = safe_normalize(sd_raw, Vec3d::UnitZ());
        d.gravity_dir  = safe_normalize(gd_raw, Vec3d(0, 0, -1));
        d.plate_normal = safe_normalize(pn_raw, Vec3d::UnitZ());

        d.q_slice_to_z = Eigen::Quaterniond::FromTwoVectors(d.slicing_dir, Vec3d::UnitZ());
        d.trafo_slice_align = Transform3d(d.q_slice_to_z);
        d.gravity_in_slice_frame = d.q_slice_to_z * d.gravity_dir;

        return d;
    }
};

} // namespace Slic3r
