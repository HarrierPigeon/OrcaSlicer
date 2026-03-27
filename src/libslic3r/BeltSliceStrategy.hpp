#pragma once

#include "libslic3r.h"
#include "Point.hpp"
#include "BeltTransform.hpp"
#include "PrintConfig.hpp"
#include "Model.hpp"

#include <limits>
#include <memory>

namespace Slic3r {

// Belt printer pre-slice transform strategy.
//
// Encapsulates the pre-remap, shear, scale, and Z-shift transforms
// that are applied to model geometry before slicing on belt printers.
// Used by PrintObjectSlice.cpp to isolate belt-specific logic from
// the slicing pipeline.
class BeltSliceStrategy
{
public:
    // Create a strategy if belt_printer is enabled; returns nullptr otherwise.
    static std::unique_ptr<BeltSliceStrategy> create(const PrintConfig &config);

    // Apply belt transforms to the slicing trafo.
    // Modifies trafo in-place: trafo = z_shift * scale * shear * pre_remap * trafo
    // Scans all model_part volumes to detect minimum Z and add z-shift if needed.
    // Sets *out_belt_min_z to the minimum Z of the mesh after transforms.
    void apply_to_trafo(Transform3d &trafo,
                        const ModelVolumePtrs &model_volumes,
                        double *out_belt_min_z) const;

private:
    explicit BeltSliceStrategy(const PrintConfig &config);

    bool        m_has_remap  = false;
    bool        m_has_shear  = false;
    bool        m_has_scale  = false;
    Transform3d m_pre_remap  = Transform3d::Identity();
    Matrix3d    m_shear      = Matrix3d::Identity();
    Matrix3d    m_scale      = Matrix3d::Identity();
};

} // namespace Slic3r
