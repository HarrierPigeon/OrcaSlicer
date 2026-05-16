#include "BeltSliceStrategy.hpp"

#include <boost/log/trivial.hpp>
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
    m_shear = BeltTransformPipeline::build_shear_matrix(config, &m_has_shear);
    m_scale = BeltTransformPipeline::build_scale_matrix(config, &m_has_scale);
    m_order = config.belt_mesh_transform_order.value;
}

void BeltSliceStrategy::apply_to_trafo(Transform3d &trafo,
                                        const ModelVolumePtrs &model_volumes,
                                        bool has_remap,
                                        double *out_belt_min_z) const
{
    // ScaleThenShear: applied to a point, scale runs first then shear (m_shear * m_scale).
    // ShearThenScale: applied to a point, shear runs first then scale (m_scale * m_shear).
    if (m_has_shear || m_has_scale) {
        Transform3d belt_xform = Transform3d::Identity();
        belt_xform.linear() = (m_order == BeltTransformOrder::ScaleThenShear)
            ? Matrix3d(m_shear * m_scale)
            : Matrix3d(m_scale * m_shear);
        trafo = belt_xform * trafo;
    }

    // Z-shift — detect if mesh clips below build plate after transforms.
    if (has_remap || m_has_shear || m_has_scale) {
        double min_z = std::numeric_limits<double>::max();
        for (const ModelVolume *mv : model_volumes) {
            if (!mv->is_model_part()) continue;
            for (const stl_vertex &v : mv->mesh().its.vertices) {
                Vec3d pt = trafo * v.cast<double>();
                min_z = std::min(min_z, pt.z());
            }
        }
        double belt_z_shift_val = (min_z < 0. && min_z != std::numeric_limits<double>::max()) ? -min_z : 0.;
        BOOST_LOG_TRIVIAL(warning) << "Belt Z-shift: min_z=" << min_z
            << " z_shift=" << belt_z_shift_val;
        if (belt_z_shift_val > 0.) {
            Transform3d z_shift = Transform3d::Identity();
            z_shift.matrix()(2, 3) = belt_z_shift_val;
            trafo = z_shift * trafo;
        }
        if (out_belt_min_z) {
            double new_val = (min_z != std::numeric_limits<double>::max()) ? min_z : 0.;
            BOOST_LOG_TRIVIAL(warning) << "[BELTRACE] write m_belt_min_z tid=" << std::this_thread::get_id()
                << " target=" << out_belt_min_z << " old=" << *out_belt_min_z << " new=" << new_val;
            *out_belt_min_z = new_val;
        }
    }
}

} // namespace Slic3r
