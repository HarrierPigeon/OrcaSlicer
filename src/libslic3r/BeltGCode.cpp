#include "BeltGCode.hpp"
#include "BeltGCodeWriter.hpp"
#include "BeltTransform.hpp"
#include "Print.hpp"

#include <limits>

namespace Slic3r {

void BeltGCode::init_belt_writer(Print &print, bool is_bbl_printers)
{
    if (!print.config().belt_printer.value)
        return;

    auto belt_writer = std::make_unique<BeltGCodeWriter>();
    belt_writer->set_is_bbl_machine(is_bbl_printers);
    belt_writer->set_belt_angle(print.config().belt_printer_angle.value);
    // Axis remap and build volume max are set by base GCode after init_belt_writer returns.
    belt_writer->set_belt_back_transform(print.config());
    m_writer = std::move(belt_writer);

    // Per-axis origin snap config.
    m_origin_snap[0] = print.config().belt_origin_snap_x.value;
    m_origin_snap[1] = print.config().belt_origin_snap_y.value;
    m_origin_snap[2] = print.config().belt_origin_snap_z.value;
    m_origin_snap_offset[0] = print.config().belt_origin_offset_x.value;
    m_origin_snap_offset[1] = print.config().belt_origin_offset_y.value;
    m_origin_snap_offset[2] = print.config().belt_origin_offset_z.value;
}

void BeltGCode::write_belt_header(GCodeOutputStream &file, const Print &print)
{
    if (!print.config().belt_printer.value)
        return;

    file.write_format("; belt_printer_angle = %.1f\n", print.config().belt_printer_angle.value);
    // Shear configs
    const auto &full_cfg = print.full_print_config();
    file.write_format("; belt_shear_x = %s\n",       full_cfg.opt_serialize("belt_shear_x").c_str());
    file.write_format("; belt_shear_x_angle = %.1f\n", print.config().belt_shear_x_angle.value);
    file.write_format("; belt_shear_x_from = %s\n",  full_cfg.opt_serialize("belt_shear_x_from").c_str());
    file.write_format("; belt_shear_y = %s\n",       full_cfg.opt_serialize("belt_shear_y").c_str());
    file.write_format("; belt_shear_y_angle = %.1f\n", print.config().belt_shear_y_angle.value);
    file.write_format("; belt_shear_y_from = %s\n",  full_cfg.opt_serialize("belt_shear_y_from").c_str());
    file.write_format("; belt_shear_z = %s\n",       full_cfg.opt_serialize("belt_shear_z").c_str());
    file.write_format("; belt_shear_z_angle = %.1f\n", print.config().belt_shear_z_angle.value);
    file.write_format("; belt_shear_z_from = %s\n",  full_cfg.opt_serialize("belt_shear_z_from").c_str());
    // Scale configs
    file.write_format("; belt_scale_x = %s\n",       full_cfg.opt_serialize("belt_scale_x").c_str());
    file.write_format("; belt_scale_x_angle = %.1f\n", print.config().belt_scale_x_angle.value);
    file.write_format("; belt_scale_y = %s\n",       full_cfg.opt_serialize("belt_scale_y").c_str());
    file.write_format("; belt_scale_y_angle = %.1f\n", print.config().belt_scale_y_angle.value);
    file.write_format("; belt_scale_z = %s\n",       full_cfg.opt_serialize("belt_scale_z").c_str());
    file.write_format("; belt_scale_z_angle = %.1f\n", print.config().belt_scale_z_angle.value);
    // Pre-slice remap configs
    file.write_format("; preslice_remap_x = %s\n", full_cfg.opt_serialize("preslice_remap_x").c_str());
    file.write_format("; preslice_remap_y = %s\n", full_cfg.opt_serialize("preslice_remap_y").c_str());
    file.write_format("; preslice_remap_z = %s\n", full_cfg.opt_serialize("preslice_remap_z").c_str());
    file.write_format("; preslice_remap_global = %d\n", print.config().preslice_remap_global.value ? 1 : 0);
    file.write_format("; belt_preslice_global = %d\n", print.config().belt_preslice_global.value ? 1 : 0);
}

void BeltGCode::on_set_origin(const PrintObject *obj, const Point &inst_shift)
{
    // Global pre-slice mode: adjust origin using computed correction.
    // Transform the origin through the belt pipeline so that
    // back_transform(T * origin) = origin (correct machine position).
    // This replaces the bbox-based axis snap with an exact formula.
    //
    // Two flags trigger this path:
    //   belt_preslice_global   — full pipeline (scale * shear * remap) is global
    //   preslice_remap_global  — only the pre-slice remap is global
    // The XY origin adjustment uses the FULL forward transform either way,
    // because the back_transform applied during G-code emission is always the
    // inverse of the full pipeline. When only the remap is configured, both
    // flags produce identical math (T == R).
    bool use_global = m_config.belt_preslice_global.value
        || (m_config.preslice_remap_global.value
            && BeltTransformPipeline::has_preslice_remap(m_config));
    if (use_global && m_config.belt_printer.value) {
        auto *belt_writer = dynamic_cast<BeltGCodeWriter*>(m_writer.get());
        if (belt_writer) {
            // Clear snap — not needed with computed corrections
            for (int a = 0; a < 3; ++a)
                belt_writer->set_origin_snap(a, false, 0., 0.);
        }

        // Adjust origin: transform through belt forward pipeline so that
        // the back-transform correctly recovers model-space positions.
        Transform3d T = BeltTransformPipeline::build_forward_transform(m_config);
        Vec2d cur_origin = this->origin();
        Vec3d origin3d(cur_origin.x(), cur_origin.y(), 0.);
        Vec3d adjusted = T.linear() * origin3d;
        this->set_origin(Vec2d(adjusted.x(), adjusted.y()));
        return;
    }

    if (!m_origin_snap[0] && !m_origin_snap[1] && !m_origin_snap[2])
        return;

    auto *belt_writer = dynamic_cast<BeltGCodeWriter*>(m_writer.get());
    if (!belt_writer)
        return;

    // Clear existing snap so to_machine_coords gives raw machine coords for bbox computation.
    for (int a = 0; a < 3; ++a)
        belt_writer->set_origin_snap(a, false, 0., 0.);

    // Reconstruct the belt pipeline transform for this object.
    Transform3d belt = BeltTransformPipeline::build_forward_transform(m_config);

    // Z-shift
    double zs = (obj->belt_min_z() < 0.) ? -obj->belt_min_z() : 0.;
    if (zs > 0.) {
        Transform3d zsh = Transform3d::Identity();
        zsh.matrix()(2, 3) = zs;
        belt = zsh * belt;
    }

    // Full transform: belt * trafo_centered
    Transform3d full = belt * obj->trafo_centered();

    // Instance shift in slicer space + global Z offset
    Vec3d shift(unscale<double>(inst_shift.x()),
                unscale<double>(inst_shift.y()),
                obj->belt_global_z_offset());

    // Compute this instance's machine-space bbox min
    BoundingBoxf3 bb = obj->model_object()->raw_bounding_box();
    Vec3d mn = bb.min.cast<double>(), mx = bb.max.cast<double>();
    Vec3d inst_min(std::numeric_limits<double>::max(),
                   std::numeric_limits<double>::max(),
                   std::numeric_limits<double>::max());
    for (int i = 0; i < 8; ++i) {
        Vec3d c((i & 1) ? mx.x() : mn.x(),
                (i & 2) ? mx.y() : mn.y(),
                (i & 4) ? mx.z() : mn.z());
        Vec3d mc = belt_writer->to_machine_coords(full * c + shift);
        for (int a = 0; a < 3; ++a)
            inst_min[a] = std::min(inst_min[a], mc[a]);
    }

    // Update writer snap for each enabled axis
    for (int a = 0; a < 3; ++a)
        belt_writer->set_origin_snap(a, m_origin_snap[a], m_origin_snap_offset[a], inst_min[a]);
}

} // namespace Slic3r
