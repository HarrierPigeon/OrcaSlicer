#include "BeltSliceStrategy.hpp"
#include "Model.hpp"

#include <boost/log/trivial.hpp>
#include <iomanip>
#include <sstream>
#include <thread>

namespace Slic3r {

std::unique_ptr<BeltSliceStrategy> BeltSliceStrategy::create(const PrintConfig &config)
{
    if (!config.belt_printer.value)
        return nullptr;
    return std::unique_ptr<BeltSliceStrategy>(new BeltSliceStrategy(config));
}

BeltSliceStrategy::BeltSliceStrategy(const PrintConfig &config)
{
    m_shear    = BeltTransformPipeline::build_shear_matrix(config, &m_has_shear);
    m_scale    = BeltTransformPipeline::build_scale_matrix(config, &m_has_scale);
    m_rotation = BeltTransformPipeline::build_rotation_matrix(config, &m_has_rotation);
    m_order    = config.belt_mesh_transform_order.value;
}

void BeltSliceStrategy::apply_to_trafo(Transform3d &trafo,
                                        const ModelVolumePtrs &model_volumes,
                                        bool has_remap,
                                        double *out_belt_min_z) const
{
    // ScaleThenShear: applied to a point, scale runs first then shear (m_shear * m_scale).
    // ShearThenScale: applied to a point, shear runs first then scale (m_scale * m_shear).
    // Rotation (if active) is applied AFTER shear/scale, matching build_forward_transform.
    if (m_has_shear || m_has_scale || m_has_rotation) {
        Matrix3d shear_scale = (m_order == BeltTransformOrder::ScaleThenShear)
            ? Matrix3d(m_shear * m_scale)
            : Matrix3d(m_scale * m_shear);
        Transform3d belt_xform = Transform3d::Identity();
        belt_xform.linear() = Matrix3d(m_rotation * shear_scale);
        trafo = belt_xform * trafo;
    }

    // Z-shift — detect if mesh clips below build plate after transforms.
    // Each mesh vertex must be brought into object space via mv->get_matrix()
    // before applying the full trafo (which is in object space).  Missing this
    // step on assemblies (where per-volume get_matrix() positions each volume
    // within the object) causes min_z to be computed against mesh-local vertex
    // coordinates rather than object-space coordinates, so volumes translated
    // along the slicer's Z axis are silently excluded from the bound check.
    if (has_remap || m_has_shear || m_has_scale || m_has_rotation) {
        // [BELT-DEBUG] Capture the incoming trafo for diagnostic logging.
        // This is the slicer-frame transform AFTER belt_xform but BEFORE z_shift.
        const Transform3d trafo_pre_shift = trafo;
        auto log_mat = [](const Matrix3d &m) {
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(4);
            ss << "[[" << m(0,0) << "," << m(0,1) << "," << m(0,2) << "],"
               << "["  << m(1,0) << "," << m(1,1) << "," << m(1,2) << "],"
               << "["  << m(2,0) << "," << m(2,1) << "," << m(2,2) << "]]";
            return ss.str();
        };
        auto log_vec3 = [](const Vec3d &v) {
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(4);
            ss << "(" << v.x() << "," << v.y() << "," << v.z() << ")";
            return ss.str();
        };
        BOOST_LOG_TRIVIAL(warning) << "[BELT-DEBUG] apply_to_trafo enter"
            << " has_shear=" << m_has_shear
            << " has_scale=" << m_has_scale
            << " has_rotation=" << m_has_rotation
            << " has_remap=" << has_remap
            << " trafo.linear=" << log_mat(trafo_pre_shift.linear())
            << " trafo.translation=" << log_vec3(trafo_pre_shift.translation())
            << " volumes=" << model_volumes.size();

        double min_z = std::numeric_limits<double>::max();
        int vol_idx = 0;
        for (const ModelVolume *mv : model_volumes) {
            if (!mv->is_model_part()) { ++vol_idx; continue; }
            Transform3d vol_trafo = trafo * mv->get_matrix();
            // [BELT-DEBUG] Per-volume bbox in mesh-frame and post-trafo slicer-frame.
            const auto &its = mv->mesh().its;
            Vec3d mesh_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
            Vec3d mesh_max(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
            Vec3d slicer_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
            Vec3d slicer_max(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
            double vol_min_z = std::numeric_limits<double>::max();
            for (const stl_vertex &v : its.vertices) {
                Vec3d vm = v.cast<double>();
                mesh_min = mesh_min.cwiseMin(vm);
                mesh_max = mesh_max.cwiseMax(vm);
                Vec3d pt = vol_trafo * vm;
                slicer_min = slicer_min.cwiseMin(pt);
                slicer_max = slicer_max.cwiseMax(pt);
                vol_min_z = std::min(vol_min_z, pt.z());
                min_z = std::min(min_z, pt.z());
            }
            BOOST_LOG_TRIVIAL(warning) << "[BELT-DEBUG]   vol[" << vol_idx
                << "] id=" << mv->id().id << " name='" << mv->name << "'"
                << " mesh_bbox_min=" << log_vec3(mesh_min) << " mesh_bbox_max=" << log_vec3(mesh_max)
                << " get_matrix.translation=" << log_vec3(mv->get_matrix().translation())
                << " slicer_bbox_min=" << log_vec3(slicer_min) << " slicer_bbox_max=" << log_vec3(slicer_max)
                << " vol_min_z=" << vol_min_z;
            ++vol_idx;
        }
        double belt_z_shift_val = (min_z < 0. && min_z != std::numeric_limits<double>::max()) ? -min_z : 0.;
        BOOST_LOG_TRIVIAL(warning) << "[BELT-DEBUG] combined min_z=" << min_z
            << " z_shift_val=" << belt_z_shift_val;
        if (belt_z_shift_val > 0.) {
            Transform3d z_shift = Transform3d::Identity();
            z_shift.matrix()(2, 3) = belt_z_shift_val;
            trafo = z_shift * trafo;
        }
        if (out_belt_min_z) {
            double new_val = (min_z != std::numeric_limits<double>::max()) ? min_z : 0.;
            BOOST_LOG_TRIVIAL(warning) << "[BELT-DEBUG] write m_belt_min_z tid=" << std::this_thread::get_id()
                << " target=" << out_belt_min_z << " old=" << *out_belt_min_z << " new=" << new_val;
            *out_belt_min_z = new_val;
        }
        BOOST_LOG_TRIVIAL(warning) << "[BELT-DEBUG] apply_to_trafo exit"
            << " final_trafo.linear=" << log_mat(trafo.linear())
            << " final_trafo.translation=" << log_vec3(trafo.translation());
    }
}

} // namespace Slic3r
