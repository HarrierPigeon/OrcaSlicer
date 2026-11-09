#include "KlipperReport.hpp"

#include <algorithm>
#include <exception>
#include <utility>

#include <nlohmann/json.hpp>

namespace Slic3r::ShakeTune {

namespace {

// Shake&Tune names its files "<graph type with the spaces removed>_<date>_<time>[_axis_X].stdata"
// (GraphCreator, RESULTS_SUBFOLDERS).
const std::pair<const char *, const char *> RECORDING_PREFIXES[] = {
    { "vibrationsprofile_", "vibrations" }, { "inputshaper_", "input_shaper" }, { "beltscomparison_", "belts" },
    { "axesmap_", "axes_map" },             { "staticfrequency_", "static_freq" },
};

std::string file_name(const std::string &path)
{
    const size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

double number_or(const nlohmann::json &obj, const char *key, double fallback)
{
    auto it = obj.find(key);
    return it != obj.end() && it->is_number() ? it->get<double>() : fallback;
}

std::string string_or(const nlohmann::json &obj, const char *key, const std::string &fallback)
{
    auto it = obj.find(key);
    return it != obj.end() && it->is_string() ? it->get<std::string>() : fallback;
}

} // namespace

bool parse_remote_recordings(const std::string &files_list_json, std::vector<RemoteRecording> &out, std::string &error)
{
    out.clear();
    try {
        const nlohmann::json j = nlohmann::json::parse(files_list_json);
        for (const nlohmann::json &f : j.at("result")) {
            const std::string path = f.at("path").get<std::string>();
            const std::string name = file_name(path);
            if (name.size() < 7 || name.compare(name.size() - 7, 7, ".stdata") != 0)
                continue;
            for (const auto &[prefix, kind] : RECORDING_PREFIXES)
                if (name.rfind(prefix, 0) == 0) {
                    RemoteRecording r;
                    r.path     = path;
                    r.kind     = kind;
                    r.modified = number_or(f, "modified", 0.);
                    r.size     = std::uint64_t(number_or(f, "size", 0.));
                    // Shake&Tune writes to a temporary file and renames it when the recording ends.
                    if (r.size > 0)
                        out.push_back(std::move(r));
                    break;
                }
        }
    } catch (const std::exception &ex) {
        error = std::string("unexpected file list from Moonraker: ") + ex.what();
        return false;
    }
    std::sort(out.begin(), out.end(), [](const RemoteRecording &a, const RemoteRecording &b) { return a.modified > b.modified; });
    return true;
}

std::optional<MotorKinematics> KlipperMotion::motor_kinematics() const
{
    if (kinematics == "corexy" || kinematics == "limited_corexy")
        return MotorKinematics::CoreXY;
    if (kinematics == "cartesian" || kinematics == "limited_cartesian" || kinematics == "corexz" || kinematics == "limited_corexz")
        return MotorKinematics::Cartesian;
    return std::nullopt;
}

bool parse_klipper_motion(const std::string &configfile_settings_json, KlipperMotion &out, std::string &error)
{
    out = KlipperMotion{};
    try {
        const nlohmann::json j        = nlohmann::json::parse(configfile_settings_json);
        const nlohmann::json &settings = j.at("result").at("status").at("configfile").at("settings");

        const nlohmann::json &printer = settings.at("printer");
        out.kinematics             = string_or(printer, "kinematics", "");
        out.max_accel              = number_or(printer, "max_accel", 0.);
        out.max_velocity           = number_or(printer, "max_velocity", 0.);
        out.square_corner_velocity = number_or(printer, "square_corner_velocity", 5.);

        if (auto is = settings.find("input_shaper"); is != settings.end()) {
            // Klipper: shaper_type applies to both axes unless shaper_type_x/_y override it; a
            // frequency of 0 leaves that axis unshaped.
            const std::string common = string_or(*is, "shaper_type", "mzv");
            auto axis = [&](const char *type_key, const char *freq_key, const char *damping_key) -> std::optional<KlipperMotion::Shaper> {
                const double freq = number_or(*is, freq_key, 0.);
                if (!(freq > 0.))
                    return std::nullopt;
                return KlipperMotion::Shaper{ string_or(*is, type_key, common), freq, number_or(*is, damping_key, 0.1) };
            };
            out.shaper_x = axis("shaper_type_x", "shaper_freq_x", "damping_ratio_x");
            out.shaper_y = axis("shaper_type_y", "shaper_freq_y", "damping_ratio_y");
        }

        if (auto st = settings.find("shaketune"); st != settings.end()) {
            out.has_shaketune      = true;
            out.shaketune_max_freq = number_or(*st, "max_freq", 200.);
            auto keep              = st->find("keep_raw_data");
            out.keep_raw_data      = keep != st->end() && keep->is_boolean() && keep->get<bool>();
        }
    } catch (const std::exception &ex) {
        error = std::string("unexpected configuration from Moonraker: ") + ex.what();
        return false;
    }
    return true;
}

} // namespace Slic3r::ShakeTune
