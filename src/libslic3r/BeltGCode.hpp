#pragma once

#include "GCode.hpp"

namespace Slic3r {

// Belt-printer-specific GCode export.
//
// Inherits from GCode and overrides virtual hooks to:
// - Create a BeltGCodeWriter instead of a plain GCodeWriter
// - Write belt configuration to the G-code header
// - Update per-axis origin snap when switching instances
// - Disable arc fitting (G2/G3 not supported on belt printers)
class BeltGCode : public GCode
{
protected:
    void init_belt_writer(Print &print, bool is_bbl_printers) override;
    void write_belt_header(GCodeOutputStream &file, const Print &print) override;
    void on_set_origin(const PrintObject *obj, const Point &inst_shift) override;
    bool should_disable_arc_fitting() const override { return true; }

private:
    // Per-axis origin snap config (set during init, used in on_set_origin).
    bool   m_origin_snap[3]        = {false, false, false};
    double m_origin_snap_offset[3] = {0., 0., 0.};
};

} // namespace Slic3r
