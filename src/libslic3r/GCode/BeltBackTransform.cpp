#include "BeltBackTransform.hpp"
#include "../BeltTransform.hpp"

namespace Slic3r {

bool BeltBackTransform::init_from_config(const PrintConfig &config)
{
    m_active  = false;
    m_inverse = Transform3d::Identity();

    if (!config.belt_printer.value || !config.gcode_back_transform.value)
        return false;

    // Require at least one active transform to proceed.
    bool has_global_shear = config.belt_shear_x_global.value ||
                            config.belt_shear_y_global.value ||
                            config.belt_shear_z_global.value;
    bool has_preslice_global = config.belt_preslice_global.value
                            || config.preslice_remap_global.value;
    if (!has_global_shear && !has_preslice_global && !BeltTransformPipeline::has_preslice_remap(config))
        return false;

    // Build the forward pipeline (scale * shear * pre_remap) and store its inverse.
    Transform3d forward = BeltTransformPipeline::build_forward_transform(config);
    if (forward.isApprox(Transform3d::Identity()))
        return false;

    m_inverse = forward.inverse();
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
