#include <catch2/catch_all.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <utility>
#include <string>
#include <vector>

#include "libslic3r/ShakeTune/InputShaperLimits.hpp"
#include "libslic3r/ShakeTune/KlipperReport.hpp"
#include "libslic3r/ShakeTune/MotorResonance.hpp"
#include "libslic3r/ShakeTune/Vibrations.hpp"

#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>

using namespace Slic3r;
using namespace Slic3r::ShakeTune;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double PI = 3.14159265358979323846;

bool any_band_contains(const SpeedBands &bands, double speed)
{
    for (const SpeedBand &b : bands)
        if (b.contains(speed))
            return true;
    return false;
}

// A vibrations profile like CREATE_VIBRATIONS_PROFILE records it, with known resonances: each
// motor's step tone gets louder around chosen motor speeds. Deterministic noise keeps it repeatable.
std::vector<Measurement> synthetic_vibrations_profile(bool corexy)
{
    struct Bump { double speed, width, gain; };
    const std::vector<Bump> bumps[2] = { { { 60., 3., 6. }, { 130., 4., 4. } }, { { 86., 3., 5. } } };
    const double            fs = 3200.;
    const size_t            n  = 2400;
    uint32_t                lcg = 12345u;
    auto noise = [&lcg]() {
        // Sum of four uniforms: roughly normal, zero mean.
        double s = 0.;
        for (int i = 0; i < 4; ++i) {
            lcg = lcg * 1664525u + 1013904223u;
            s += double(lcg) / 4294967296. - 0.5;
        }
        return s * 40.;
    };
    std::vector<Measurement> out;
    const double             angles[2] = { corexy ? 45. : 0., corexy ? 135. : 90. };
    for (int motor = 0; motor < 2; ++motor) {
        const double rad = angles[motor] * PI / 180.;
        for (int sp = 2; sp <= 200; sp += 2) {
            double gain = 1.;
            for (const Bump &b : bumps[motor])
                gain += b.gain * std::exp(-0.5 * std::pow((sp - b.speed) / b.width, 2.));
            const double f_step = sp * (200. / 32.) / 4.;
            Measurement  m;
            char         name[64];
            std::snprintf(name, sizeof(name), "vib_an%d_00sp%d_00_20261108_120000", int(angles[motor]), sp);
            m.name = name;
            for (size_t i = 0; i < n; ++i) {
                const double t     = double(i) / fs;
                const double tone  = 30. * gain * (std::sin(2. * PI * f_step * t) + 0.3 * std::sin(4. * PI * f_step * t));
                const double frame = 40. * std::sin(2. * PI * 48. * t);
                m.samples.push_back({ 1000. + t, (tone + frame) * std::cos(rad) + noise(), (tone + frame) * std::sin(rad) + noise(),
                                      0.2 * tone + noise() + 9810. });
            }
            out.push_back(std::move(m));
        }
    }
    return out;
}

} // namespace

TEST_CASE("Input shaper smoothing and max accel match Klipper's published calibration figures", "[ShakeTune]")
{
    // From Klipper's docs/Measuring_Resonances.md: "Fitted shaper '<name>' frequency = <f> Hz
    // (..., smoothing ~= <s>)" and "suggested max_accel <= <a>", at the defaults it calibrates with
    // (damping ratio 0.1, square corner velocity 5, smoothing quoted at 5000 mm/s²). Only ZV and
    // MZV: Klipper redefined the EI family in 2024 (5da026a33) and the doc's EI figures predate it.
    struct Row { const char *name; double freq, smoothing, max_accel; };
    const Row row = GENERATE(Row{ "zv", 34.4, 0.132, 4500 }, Row{ "mzv", 34.6, 0.170, 3500 },
                             Row{ "zv", 57.8, 0.053, 13000 }, Row{ "mzv", 34.8, 0.168, 3600 });
    CAPTURE(row.name, row.freq);
    const std::optional<InputShaper> shaper = make_input_shaper(row.name, row.freq, 0.1);
    REQUIRE(shaper.has_value());
    CHECK_THAT(input_shaper_smoothing(*shaper, 5000., 5.), WithinAbs(row.smoothing, 0.0005));
    CHECK_THAT(round_max_accel(input_shaper_max_accel(*shaper, 5.)), WithinAbs(row.max_accel, 0.5));
}

TEST_CASE("Input shaper max accel is where smoothing reaches Klipper's 0.12 mm target", "[ShakeTune]")
{
    // At one frequency, each shaper in this list smooths more than the one before it, so it
    // allows less acceleration.
    const char *names[] = { "zv", "mzv", "ei", "2hump_ei", "3hump_ei" };
    double      previous = INFINITY;
    for (const char *name : names) {
        CAPTURE(name);
        const std::optional<InputShaper> shaper = make_input_shaper(name, 50., 0.1);
        REQUIRE(shaper.has_value());
        const double accel = input_shaper_max_accel(*shaper, 5.);
        CHECK_THAT(input_shaper_smoothing(*shaper, accel, 5.), WithinAbs(0.12, 1e-6));
        CHECK(accel < previous);
        previous = accel;
    }
}

TEST_CASE("Unknown input shapers have no limits", "[ShakeTune]")
{
    CHECK_FALSE(make_input_shaper("smooth_zv", 50., 0.1).has_value());
    CHECK_FALSE(make_input_shaper("mzv", 0., 0.1).has_value());
}

TEST_CASE("Speed bands parse in any order, merge overlaps and print back", "[ShakeTune]")
{
    SpeedBands  bands;
    std::string error;
    REQUIRE(parse_speed_bands(" 96-104, 45 - 52;50-55 ", bands, &error));
    REQUIRE(bands.size() == 2);
    CHECK_THAT(bands[0].lo, WithinAbs(45., 1e-9));
    CHECK_THAT(bands[0].hi, WithinAbs(55., 1e-9));
    CHECK_THAT(bands[1].lo, WithinAbs(96., 1e-9));
    CHECK_THAT(bands[1].hi, WithinAbs(104., 1e-9));
    CHECK(format_speed_bands(bands) == "45-55, 96-104");

    CHECK(parse_speed_bands("", bands, &error));
    CHECK(bands.empty());
    CHECK_FALSE(parse_speed_bands("45", bands, &error));
    CHECK_FALSE(parse_speed_bands("52-45", bands, &error));
    CHECK_FALSE(parse_speed_bands("a-b", bands, &error));
}

TEST_CASE("Safe speed drops to the lower edge of the band a motor sits in", "[ShakeTune]")
{
    // CoreXY: a move along 45 degrees drives only motor A at the full toolhead speed.
    const std::array<SpeedBands, 2> bands{ SpeedBands{ { 55., 65. } }, SpeedBands{} };
    const double                    safe = resonance_safe_speed(MotorKinematics::CoreXY, bands, 60., 1., 1.);
    CHECK(safe < 55.);
    CHECK_THAT(safe, WithinAbs(55., 1e-3));
    // Outside the band, or along 135 degrees where motor A stands still: unchanged.
    CHECK_THAT(resonance_safe_speed(MotorKinematics::CoreXY, bands, 70., 1., 1.), WithinAbs(70., 1e-12));
    CHECK_THAT(resonance_safe_speed(MotorKinematics::CoreXY, bands, 60., -1., 1.), WithinAbs(60., 1e-12));
    // Cartesian X runs at |cos| of the toolhead speed: 100 mm/s at 53.13 degrees puts it at 60.
    const double cart = resonance_safe_speed(MotorKinematics::Cartesian, bands, 100., 0.6, 0.8);
    CHECK_THAT(cart * 0.6, WithinAbs(55., 1e-3));
}

TEST_CASE("Safe speed gives up rather than more than halve the speed", "[ShakeTune]")
{
    const std::array<SpeedBands, 2> bands{ SpeedBands{ { 10., 100. } }, SpeedBands{} };
    CHECK_THAT(resonance_safe_speed(MotorKinematics::Cartesian, bands, 60., 1., 0.), WithinAbs(60., 1e-12));
}

TEST_CASE("Safe speed clears both motors at once", "[ShakeTune]")
{
    // Cartesian at 45 degrees: both motors run at 0.7071 of the toolhead speed. Motor X's band
    // pushes 60 mm/s down to 50/0.7071 = 70.7... which is above 60, so take motor Y's edge.
    const std::array<SpeedBands, 2> bands{ SpeedBands{ { 35., 45. } }, SpeedBands{ { 38., 50. } } };
    const double                    v = resonance_safe_speed(MotorKinematics::Cartesian, bands, 60., 1., 1.);
    const std::array<double, 2>     f = motor_speed_factors(MotorKinematics::Cartesian, 1., 1.);
    CHECK_FALSE(any_band_contains(bands[0], v * f[0]));
    CHECK_FALSE(any_band_contains(bands[1], v * f[1]));
    CHECK(v < 60.);
}

TEST_CASE("Vibrations recording names give the angle and speed", "[ShakeTune]")
{
    double angle = 0., speed = 0.;
    REQUIRE(parse_vibrations_measurement_name("vib_an45_00sp120_00_20261108_120000", angle, speed));
    CHECK_THAT(angle, WithinAbs(45., 1e-9));
    CHECK_THAT(speed, WithinAbs(120., 1e-9));
    REQUIRE(parse_vibrations_measurement_name("vib_an135_00sp2_50", angle, speed));
    CHECK_THAT(speed, WithinAbs(2.5, 1e-9));
    CHECK_FALSE(parse_vibrations_measurement_name("inputshaper_X_20261108", angle, speed));
}

TEST_CASE("A .stdata file decodes into named recordings", "[ShakeTune]")
{
    std::vector<Measurement> m;
    std::string              error;
    // Two recordings, after a record without samples that is not one. The first recording also carries
    // the "parameters" and "machine" records a newer Shake&Tune writes; of those only the kinematics are kept.
    REQUIRE(load_stdata(std::string(TEST_DATA_DIR) + "/shaketune/tiny.stdata", m, error));
    REQUIRE(m.size() == 2);
    CHECK(m[0].name == "vib_an45_00sp2_00_20261108_120000");
    REQUIRE(m[0].samples.size() == 2);
    CHECK_THAT(m[0].samples[1][1], WithinAbs(11., 1e-9));
    CHECK(m[0].kinematics == "corexy");
    CHECK(m[1].samples.size() == 1);
    CHECK(m[1].kinematics.empty());

    CHECK_FALSE(parse_stdata("not zstd at all", m, error));
    CHECK_FALSE(error.empty());
}

TEST_CASE("Frequency response peaks at the frequency of a pure tone", "[ShakeTune]")
{
    std::vector<std::array<double, 4>> samples;
    for (size_t i = 0; i < 6400; ++i) {
        const double t = double(i) / 3200.;
        samples.push_back({ t, 100. * std::sin(2. * PI * 125. * t), 0., 0. });
    }
    const std::optional<FrequencyResponse> fr = frequency_response(samples);
    REQUIRE(fr.has_value());
    const size_t peak = size_t(std::max_element(fr->psd_sum.begin(), fr->psd_sum.end()) - fr->psd_sum.begin());
    CHECK_THAT(fr->freqs[peak], WithinAbs(125., 2.));
}

TEST_CASE("Vibrations analysis finds each motor's resonant speeds", "[ShakeTune]")
{
    const bool               corexy = GENERATE(true, false);
    std::string              error;
    const auto               profile = analyze_vibrations(synthetic_vibrations_profile(corexy), DEFAULT_VIBRATIONS_MAX_FREQ, error);
    INFO(error);
    REQUIRE(profile.has_value());
    CHECK(profile->kinematics == (corexy ? MotorKinematics::CoreXY : MotorKinematics::Cartesian));
    CHECK(profile->measured_speeds.size() == 100);
    // The resonances were put at 60 and 130 mm/s on the first motor and 86 mm/s on the second.
    CHECK(any_band_contains(profile->motor_bands[0], 60.));
    CHECK(any_band_contains(profile->motor_bands[0], 130.));
    CHECK(any_band_contains(profile->motor_bands[1], 86.));
    CHECK_FALSE(any_band_contains(profile->motor_bands[0], 86.));
    CHECK_FALSE(any_band_contains(profile->motor_bands[1], 60.));
    for (const SpeedBands &bands : profile->motor_bands)
        for (const SpeedBand &b : bands)
            CHECK(b.hi - b.lo < 20.);
    // The direction-averaged profile flags speeds to avoid and leaves quiet ranges between them.
    CHECK_FALSE(profile->peak_speeds.empty());
    CHECK_FALSE(profile->good_speed_ranges.empty());
    CHECK(profile->symmetry_pct > 0.);
}

TEST_CASE("Kinematics recorded in a vibrations profile tell which directions are the motors'", "[ShakeTune]")
{
    // Recorded along both pairs of motor directions, with the CoreXY motors' resonances along 45 and 135 degrees.
    std::vector<Measurement> measurements = synthetic_vibrations_profile(true);
    const size_t             recorded     = measurements.size();
    for (size_t i = 0; i < recorded; ++i) {
        Measurement m = measurements[i];
        m.name.replace(0, m.name.find("_00sp"), m.name.compare(0, 8, "vib_an45") == 0 ? "vib_an90" : "vib_an0");
        measurements.push_back(std::move(m));
    }
    std::string error;
    CHECK_FALSE(analyze_vibrations(measurements, DEFAULT_VIBRATIONS_MAX_FREQ, error).has_value());

    // The file's own record outweighs printer settings that say otherwise.
    measurements.front().kinematics = "corexy";
    const auto profile = analyze_vibrations(measurements, DEFAULT_VIBRATIONS_MAX_FREQ, error, MotorKinematics::Cartesian);
    INFO(error);
    REQUIRE(profile.has_value());
    CHECK(profile->kinematics == MotorKinematics::CoreXY);
    CHECK(any_band_contains(profile->motor_bands[0], 60.));

    // Kinematics the analysis doesn't know leave the choice to the settings.
    measurements.front().kinematics = "delta";
    const auto cartesian = analyze_vibrations(measurements, DEFAULT_VIBRATIONS_MAX_FREQ, error, MotorKinematics::Cartesian);
    REQUIRE(cartesian.has_value());
    CHECK(cartesian->kinematics == MotorKinematics::Cartesian);
}

namespace {

// Trimmed from a Voron Micron running Kalico and Shake&Tune v6.0.0.
const char *MICRON_CONFIGFILE_SETTINGS = R"({"result": {"eventtime": 1.0, "status": {"configfile": {"settings": {
    "printer": {"max_velocity": 200.0, "max_z_velocity": 25.0, "max_accel": 4700.0, "minimum_cruise_ratio": 0.5,
                "square_corner_velocity": 5.0, "kinematics": "corexy", "max_z_accel": 350.0},
    "input_shaper": {"shaper_type": "mzv", "shaper_type_x": "mzv", "damping_ratio_x": 0.104, "shaper_freq_x": 69.0,
                     "shaper_type_y": "mzv", "damping_ratio_y": 0.075, "shaper_freq_y": 40.4},
    "shaketune": {"result_folder": "~/printer_data/config/ShakeTune_results", "number_of_results_to_keep": 10,
                  "keep_raw_data": true, "max_freq": 200.0, "dpi": 150, "measurements_chunk_size": 2, "timeout": 600,
                  "show_macros_in_webui": true}}}}}})";

const char *MICRON_FILE_LIST = R"({"result": [
    {"path": "ShakeTune_results/vibrations/vibrationsprofile_20260825_211159.png", "modified": 1787689542.4750211, "size": 1999328, "permissions": "rw"},
    {"path": "ShakeTune_results/vibrations/vibrationsprofile_20261109_030627.stdata", "modified": 1794194337.8181436, "size": 14205475, "permissions": "rw"},
    {"path": "ShakeTune_results/vibrations/vibrationsprofile_20261109_030627.png", "modified": 1794194409.2603612, "size": 1818586, "permissions": "rw"},
    {"path": "ShakeTune_results/input_shaper/inputshaper_20261109_032010_axis_X.stdata", "modified": 1794194573.5762618, "size": 4980164, "permissions": "rw"},
    {"path": "ShakeTune_results/input_shaper/snt_tmp-0603170c.stdata", "modified": 1794194585.5639627, "size": 0, "permissions": "rw"},
    {"path": "printer.cfg", "modified": 1794190000.0, "size": 9000, "permissions": "rw"}]})";

} // namespace

TEST_CASE("Moonraker's file list yields Shake&Tune recordings, newest first", "[ShakeTune]")
{
    std::vector<RemoteRecording> recordings;
    std::string                  error;
    REQUIRE(parse_remote_recordings(MICRON_FILE_LIST, recordings, error));
    // Graphs, the empty temporary file and printer.cfg are skipped.
    REQUIRE(recordings.size() == 2);
    CHECK(recordings[0].kind == "input_shaper");
    CHECK(recordings[1].kind == "vibrations");
    CHECK(recordings[1].path == "ShakeTune_results/vibrations/vibrationsprofile_20261109_030627.stdata");
    CHECK(recordings[1].size == 14205475u);
}

TEST_CASE("Klipper's saved configuration gives the motion limits and input shapers", "[ShakeTune]")
{
    KlipperMotion motion;
    std::string   error;
    REQUIRE(parse_klipper_motion(MICRON_CONFIGFILE_SETTINGS, motion, error));
    CHECK(motion.kinematics == "corexy");
    CHECK(motion.motor_kinematics() == MotorKinematics::CoreXY);
    CHECK_THAT(motion.max_accel, WithinAbs(4700., 1e-9));
    CHECK_THAT(motion.square_corner_velocity, WithinAbs(5., 1e-9));
    REQUIRE(motion.shaper_x.has_value());
    REQUIRE(motion.shaper_y.has_value());
    CHECK(motion.shaper_y->type == "mzv");
    CHECK_THAT(motion.shaper_y->freq, WithinAbs(40.4, 1e-9));
    CHECK_THAT(motion.shaper_y->damping_ratio, WithinAbs(0.075, 1e-9));
    CHECK(motion.has_shaketune);
    CHECK(motion.keep_raw_data);

    // The Y shaper is the limiting one: a lower frequency smooths more at the same acceleration.
    const auto x = make_input_shaper(motion.shaper_x->type, motion.shaper_x->freq, motion.shaper_x->damping_ratio);
    const auto y = make_input_shaper(motion.shaper_y->type, motion.shaper_y->freq, motion.shaper_y->damping_ratio);
    REQUIRE(x.has_value());
    REQUIRE(y.has_value());
    CHECK(input_shaper_max_accel(*y, motion.square_corner_velocity) < input_shaper_max_accel(*x, motion.square_corner_velocity));
}

namespace {

// Per-recording energies of a CREATE_VIBRATIONS_PROFILE run, as vibrations_energy() computes them
// from the .stdata file: {"<angle>": [[speed, energy], ...]}.
VibrationsEnergy load_vibrations_energy(const std::string &file)
{
    boost::nowide::ifstream in(std::string(TEST_DATA_DIR) + "/shaketune/" + file);
    REQUIRE(in.good());
    const nlohmann::json json = nlohmann::json::parse(in);
    VibrationsEnergy     energy;
    for (const auto &[angle, rows] : json.items())
        for (const auto &row : rows)
            energy[std::stod(angle)][row[0].get<double>()] = row[1].get<double>();
    return energy;
}

} // namespace

TEST_CASE("A vibrations profile recorded along more directions than the motors' is analysed along the motors'", "[ShakeTune]")
{
    // CREATE_VIBRATIONS_PROFILE ANGLES=... records along other directions as well, possibly at other speeds.
    const VibrationsEnergy motors = load_vibrations_energy("trident_vibrations_2-200mms.json");
    std::string            error;
    const auto             expected = analyze_vibrations_energy(motors, error);
    REQUIRE(expected.has_value());
    const auto matches = [&expected](const std::optional<VibrationsProfile> &p) {
        REQUIRE(p.has_value());
        CHECK(p->kinematics == expected->kinematics);
        CHECK(p->measured_speeds == expected->measured_speeds);
        CHECK(p->peak_speeds == expected->peak_speeds);
        CHECK(p->symmetry_pct == expected->symmetry_pct);
    };

    // Another direction, at speeds the motors were not measured at.
    VibrationsEnergy more = motors;
    for (const auto &[speed, energy] : motors.at(45.))
        more[30.][speed + 0.5] = 3. * energy;
    matches(analyze_vibrations_energy(more, error));

    // The other pair of motor directions as well: only the printer's kinematics tells which pair is the motors'.
    for (const auto &[speed, energy] : motors.at(45.)) {
        more[0.][speed]  = 2. * energy;
        more[90.][speed] = 5. * energy;
    }
    error.clear();
    CHECK_FALSE(analyze_vibrations_energy(more, error).has_value());
    CHECK(error.find("kinematics") != std::string::npos);
    matches(analyze_vibrations_energy(more, error, MotorKinematics::CoreXY));
    const auto cartesian = analyze_vibrations_energy(more, error, MotorKinematics::Cartesian);
    REQUIRE(cartesian.has_value());
    CHECK(cartesian->kinematics == MotorKinematics::Cartesian);
    CHECK(cartesian->motor_angles == std::array<double, 2>{ 0., 90. });
    CHECK(cartesian->motor_energy[1][0] == 5. * motors.at(45.).begin()->second);

    // Settings that disagree with the recordings give way to them when the recordings cover one pair only.
    matches(analyze_vibrations_energy(motors, error, MotorKinematics::Cartesian));
}

TEST_CASE("Vibrations analysis of a real printer matches Shake&Tune's own results", "[ShakeTune]")
{
    // Two runs on a Voron Trident (CoreXY, Kalico, Shake&Tune v6.0.0), at 45 and 135 degrees. The
    // expected values are the ones Shake&Tune printed on its graphs for the same runs.
    struct Run
    {
        const char                             *file;
        std::vector<double>                     peaks;
        std::vector<std::pair<double, double>>  good_speeds;
    };
    const Run run = GENERATE(Run{ "trident_vibrations_2-200mms.json",
                                  { 32.1, 44.3, 69.8, 86.6, 125.6, 137.2 },
                                  { { 2.0, 29.1 }, { 35.4, 40.0 }, { 48.6, 62.5 }, { 95.5, 122.7 }, { 128.9, 134.2 }, { 140.5, 200.0 } } },
                             Run{ "trident_vibrations_2-599mms.json",
                                  { 44.8, 70.2, 87.1, 137.9, 174.3, 272.9, 409.3 },
                                  { { 2.0, 40.3 }, { 49.8, 63.2 }, { 97.1, 133.4 }, { 143.9, 171.3 }, { 177.8, 269.9 }, { 276.4, 406.3 }, { 412.8, 599.0 } } });
    CAPTURE(run.file);
    std::string error;
    const auto  p = analyze_vibrations_energy(load_vibrations_energy(run.file), error);
    REQUIRE(p.has_value());
    CHECK(p->kinematics == MotorKinematics::CoreXY);
    CHECK_THAT(p->symmetry_pct, WithinAbs(100., 0.05));
    REQUIRE(p->peak_speeds.size() == run.peaks.size());
    for (size_t i = 0; i < run.peaks.size(); ++i)
        CHECK_THAT(p->peak_speeds[i], WithinAbs(run.peaks[i], 0.05));
    REQUIRE(p->good_speed_ranges.size() == run.good_speeds.size());
    for (size_t i = 0; i < run.good_speeds.size(); ++i) {
        CHECK_THAT(p->good_speed_ranges[i].lo, WithinAbs(run.good_speeds[i].first, 0.05));
        CHECK_THAT(p->good_speed_ranges[i].hi, WithinAbs(run.good_speeds[i].second, 0.05));
    }

    // Not in Shake&Tune: both motors resonate at about 30 and 60 mm/s, which a move along X or Y
    // (both motors at v/sqrt(2)) shows as Shake&Tune's peaks at 44 and 87 mm/s.
    for (size_t m = 0; m < 2; ++m) {
        CAPTURE(m, format_speed_bands(p->motor_bands[m]));
        CHECK(any_band_contains(p->motor_bands[m], 31.));
        CHECK(any_band_contains(p->motor_bands[m], 61.));
        // Above 300 mm/s the measured moves are mostly acceleration and their energy climbs with
        // speed; that climb is no resonance and gets no band.
        CHECK_FALSE(std::any_of(p->motor_bands[m].begin(), p->motor_bands[m].end(), [](const SpeedBand &b) { return b.hi > 300.; }));
    }
    // Motor A also resonates around 120 mm/s, in both runs, even though the second one is far louder
    // at its top speeds.
    CHECK(any_band_contains(p->motor_bands[0], 124.));
}

