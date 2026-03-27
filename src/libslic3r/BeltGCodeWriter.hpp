#pragma once

#include "GCodeWriter.hpp"
#include "GCode/BeltBackTransform.hpp"

namespace Slic3r {

// Belt-printer-specific GCode writer.
//
// Inherits from GCodeWriter and overrides movement methods to apply
// coordinate transformation (back-transform, axis remap, origin snap)
// and emit coupled XYZ moves (Y and Z are coupled due to belt tilt).
class BeltGCodeWriter : public GCodeWriter
{
public:
    BeltGCodeWriter() : GCodeWriter() {}

    // Belt configuration
    void set_belt_angle(double angle_deg);
    bool is_belt_printer() const { return m_belt_angle_rad != 0.; }
    void set_axis_remap(int rx, int ry, int rz);
    void set_build_volume_max(const Vec3d &max);
    void set_belt_back_transform(const PrintConfig &config);
    void set_origin_snap(int axis, bool enable, double offset, double bbox_min);
    Vec3d to_machine_coords(const Vec3d &pos) const;

    // Overridden movement methods
    std::string travel_to_xy(const Vec2d &point, const std::string &comment = std::string()) override;
    std::string travel_to_xyz(const Vec3d &point, const std::string &comment = std::string(), bool force_z = false) override;
    std::string extrude_to_xy(const Vec2d &point, double dE, const std::string &comment = std::string(), bool force_no_extrusion = false) override;
    std::string extrude_to_xyz(const Vec3d &point, double dE, const std::string &comment = std::string(), bool force_no_extrusion = false) override;
    std::string lazy_lift(LiftType lift_type = LiftType::NormalLift, bool spiral_vase = false) override;
    std::string eager_lift(const LiftType type) override;

protected:
    std::string _travel_to_z(double z, const std::string &comment) override;

private:
    double          m_belt_angle_rad = 0.;
    int             m_remap_x = 0;
    int             m_remap_y = 1;
    int             m_remap_z = 2;
    Vec3d           m_build_vol_max = Vec3d::Zero();
    BeltBackTransform m_belt_back_transform;
    bool            m_origin_snap[3]     = {false, false, false};
    double          m_origin_offset[3]   = {0., 0., 0.};
    double          m_origin_bbox_min[3] = {0., 0., 0.};
};

} // namespace Slic3r
