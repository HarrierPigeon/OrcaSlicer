#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "MotorResonance.hpp"

namespace Slic3r::ShakeTune {

// A raw Shake&Tune recording found on the printer through Moonraker's "config" file root.
struct RemoteRecording
{
    // Path inside the config root, e.g. "ShakeTune_results/vibrations/vibrationsprofile_20261109_030627.stdata".
    std::string   path;
    // Which Shake&Tune command made it, from the file name: "vibrations", "input_shaper", "belts",
    // "axes_map" or "static_freq".
    std::string   kind;
    double        modified{ 0. }; // Unix time
    std::uint64_t size{ 0 };
};

// The .stdata files in a /server/files/list?root=config reply, newest first. Shake&Tune's
// result_folder can be any folder under the config root, so files are recognised by name, not place.
bool parse_remote_recordings(const std::string &files_list_json, std::vector<RemoteRecording> &out, std::string &error);

// The parts of Klipper's saved configuration the import uses, from a
// /printer/objects/query?configfile=settings reply.
struct KlipperMotion
{
    struct Shaper
    {
        std::string type;
        double      freq{ 0. };
        double      damping_ratio{ 0.1 };
    };

    std::string           kinematics;
    double                max_accel{ 0. };
    double                max_velocity{ 0. };
    double                square_corner_velocity{ 5. };
    std::optional<Shaper> shaper_x;
    std::optional<Shaper> shaper_y;
    // From [shaketune]; absent when the plugin is not configured.
    bool                  has_shaketune{ false };
    bool                  keep_raw_data{ false };
    double                shaketune_max_freq{ 200. };

    // Motor layout for the given Klipper kinematics, if Shake&Tune's vibrations profile supports it.
    std::optional<MotorKinematics> motor_kinematics() const;
};

bool parse_klipper_motion(const std::string &configfile_settings_json, KlipperMotion &out, std::string &error);

} // namespace Slic3r::ShakeTune
