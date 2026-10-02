#ifndef slic3r_CoolingBuffer_hpp_
#define slic3r_CoolingBuffer_hpp_

#include "../libslic3r.h"
#include <map>
#include <string>
#include <cfloat>

namespace Slic3r {

class GCode;
class Layer;
struct PerExtruderAdjustments;

// A standalone G-code filter, to control cooling of the print.
// The G-code is processed per layer. Once a layer is collected, fan start / stop commands are edited
// and the print is modified to stretch over a minimum layer time.
//
// The simple it sounds, the actual implementation is significantly more complex.
// Namely, for a multi-extruder print, each material may require a different cooling logic.
// For example, some materials may not like to print too slowly, while with some materials
// we may slow down significantly.
//
class CoolingBuffer {
public:
    CoolingBuffer(GCode &gcodegen);
    void        reset(const Vec3d &position);
    void        set_current_extruder(unsigned int extruder_id, unsigned int nozzle_id) { m_current_extruder = extruder_id; m_current_nozzle = nozzle_id; }
    std::string process_layer(std::string &&gcode, size_t layer_id, bool flush);

private:
	CoolingBuffer& operator=(const CoolingBuffer&) = delete;
    std::vector<PerExtruderAdjustments> parse_layer_gcode(const std::string &gcode, std::vector<float> &current_pos) const;
    float       calculate_layer_slowdown(std::vector<PerExtruderAdjustments> &per_extruder_adjustments);
    // Apply slow down over G-code lines stored in per_extruder_adjustments, enable fan if needed.
    // Returns the adjusted G-code.
    std::string apply_layer_cooldown(const std::string &gcode, size_t layer_id, float layer_time, std::vector<PerExtruderAdjustments> &per_extruder_adjustments);

    // Belt printers: turn the ";_BELT_BAND:<n>" tags GCode::_extrude() leaves in the
    // layer's G-code into part-fan changes, so the fan follows a path's height above the
    // belt rather than the slicing layer index, and strip the tags.
    std::string apply_belt_band_fan(std::string &&gcode_in, float layer_time, unsigned int extruder_at_start);

    // Pure helper: compute the main fan speed for a given effective layer
    // index (layer-id units, mapped through the plane evaluator) and the
    // current extruder.  Mirrors the inline logic in the change_extruder_set_fan
    // lambda but is callable from per-line code.
    int compute_main_fan_speed(int effective_layer_id, float layer_time,
                               unsigned int extruder_id) const;

    // G-code snippet cached for the support layers preceding an object layer.
    std::string                 m_gcode;
    // Internal data.
    // BBS: X,Y,Z,E,F,I,J
    std::vector<char>           m_axis;
    std::vector<float>          m_current_pos;
    // Current known fan speed or -1 if not known yet.
    int                         m_fan_speed;
    int                         m_additional_fan_speed;
    // Cached from GCodeWriter.
    // Printing extruder IDs, zero based.
    std::vector<unsigned int>   m_extruder_ids;
    // Highest of m_extruder_ids plus 1.
    unsigned int                m_num_extruders { 0 };
    const std::string           m_toolchange_prefix;
    // Referencs GCode::m_config, which is FullPrintConfig. While the PrintObjectConfig slice of FullPrintConfig is being modified,
    // the PrintConfig slice of FullPrintConfig is constant, thus no thread synchronization is required.
    const PrintConfig          &m_config;
    unsigned int                m_current_extruder;
    unsigned int                m_current_nozzle;
    //BBS: current fan speed
    int                         m_current_fan_speed;
    // Belt band pass state, kept across layers. The part fan as this pass last saw or set
    // it (percent, -1 unknown), and the last value the layer-level cooling asked for.
    int                         m_belt_band_fan       = -1;
    int                         m_belt_band_layer_fan = -1;
};

}

#endif
