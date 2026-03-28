#include "BeltSliceStrategy.hpp"

#include <boost/log/trivial.hpp>

namespace Slic3r {

std::unique_ptr<BeltSliceStrategy> BeltSliceStrategy::create(const PrintConfig &config)
{
    if (!config.belt_printer.value)
        return nullptr;
    return std::unique_ptr<BeltSliceStrategy>(new BeltSliceStrategy(config));
}

BeltSliceStrategy::BeltSliceStrategy(const PrintConfig &config)
{
    m_has_remap = BeltTransformPipeline::has_preslice_remap(config);
    if (m_has_remap)
        m_pre_remap = BeltTransformPipeline::build_preslice_remap(config);

    m_shear = BeltTransformPipeline::build_shear_matrix(config, &m_has_shear);
    m_scale = BeltTransformPipeline::build_scale_matrix(config, &m_has_scale);
}

void BeltSliceStrategy::apply_to_trafo(Transform3d &trafo,
                                        const ModelVolumePtrs &model_volumes,
                                        double *out_belt_min_z) const
{
    // Step 1: Pre-slice axis remap.
    if (m_has_remap)
        trafo = m_pre_remap * trafo;

    // Step 2: Shear + scale.
    if (m_has_shear || m_has_scale) {
        Transform3d belt_xform = Transform3d::Identity();
        belt_xform.linear() = m_scale * m_shear;
        trafo = belt_xform * trafo;
    }

    // Step 3: Z-shift — detect if mesh clips below build plate after transforms.
    if (m_has_remap || m_has_shear || m_has_scale) {
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
        if (out_belt_min_z)
            *out_belt_min_z = (min_z != std::numeric_limits<double>::max()) ? min_z : 0.;
    }
}

} // namespace Slic3r
