// Shake&Tune machine vibrations analysis, translated from Shake&Tune
// (Copyright (C) 2024 Félix Boisselier, GPL-3.0) and Klipper (Copyright (C) 2020-2024 Dmitry
// Butyugin, GNU GPLv3). See Vibrations.cpp for the sources and their notices.

#pragma once

#include <array>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "MotorResonance.hpp"

namespace Slic3r::ShakeTune {

// One accelerometer recording from a Shake&Tune .stdata file.
struct Measurement
{
    std::string                        name;
    // time [s], acceleration x, y, z [mm/s²]
    std::vector<std::array<double, 4>> samples;
};

// A .stdata file is a zstd stream of JSON lines, one {"name": ..., "samples": [[t, x, y, z], ...]}
// object per recording; lines without "samples" are skipped. Shake&Tune keeps these next to its
// graphs only with keep_raw_data: True.
bool parse_stdata(const std::string &compressed, std::vector<Measurement> &out, std::string &error);
bool load_stdata(const std::string &path, std::vector<Measurement> &out, std::string &error);

// Power spectral density of one recording, summed over the three accelerometer axes.
struct FrequencyResponse
{
    std::vector<double> freqs;
    std::vector<double> psd_sum;
};
// Klipper's ShaperCalibrate.calc_freq_response: Welch's method with a Kaiser window, windows of
// ~0.5 s rounded up to a power of two, 50% overlap. Nullopt when the recording is shorter than
// one window.
std::optional<FrequencyResponse> frequency_response(const std::vector<std::array<double, 4>> &samples);

// CREATE_VIBRATIONS_PROFILE names each recording "vib_an<angle>sp<speed>_<time>" with the dots of
// both numbers written as underscores, e.g. "vib_an45_00sp120_00_20261108_120000".
bool parse_vibrations_measurement_name(const std::string &name, double &angle, double &speed);

// A range on the speed (mm/s) or angle (degrees) axis, with its mean vibration level as a
// percentage of the curve's maximum.
struct ProfileRange
{
    double lo{ 0. };
    double hi{ 0. };
    double mean_energy_pct{ 0. };
};

// The result of Shake&Tune's machine vibrations analysis, plus per-motor resonance bands.
struct VibrationsProfile
{
    MotorKinematics       kinematics{ MotorKinematics::Cartesian };
    // Directions the two motors were measured along: 0/90 (Cartesian, CoreXZ) or 45/135 (CoreXY).
    std::array<double, 2> motor_angles{ 0., 90. };

    // The raw measurement grid: vibration energy of each motor at each speed it was driven at.
    std::vector<double>                measured_speeds;
    std::array<std::vector<double>, 2> motor_energy;

    // Shake&Tune's "global speed energy profile", on a grid six times finer than the measurements.
    // The vibration metric is high where vibrations are strong or differ a lot between directions.
    std::vector<double>       speeds;
    std::vector<double>       vibration_metric;
    std::vector<double>       max_energy;
    std::vector<double>       peak_speeds;
    std::vector<ProfileRange> good_speed_ranges; // ordered by speed

    // Shake&Tune's "polar angle energy profile": energy per direction of travel, 0..360 degrees.
    std::vector<double>       angles;
    std::vector<double>       angle_energy;
    std::vector<ProfileRange> good_angle_ranges; // quietest first
    // How alike the two halves of the direction/speed map are, in percent (100 = symmetric).
    double                    symmetry_pct{ 0. };

    // Not in Shake&Tune: each motor's energy curve on `speeds`, its resonance peaks and the
    // speed bands around them, for per-motor speed avoidance.
    std::array<std::vector<double>, 2> motor_curve;
    std::array<std::vector<double>, 2> motor_peak_speeds;
    std::array<SpeedBands, 2>          motor_bands;
};

// Shake&Tune integrates the vibrations PSDs up to five times its max_freq setting (200 Hz by default).
constexpr double DEFAULT_VIBRATIONS_MAX_FREQ = 1000.;

// Vibration energy of each recording, keyed by the direction (degrees) and speed (mm/s) it was
// recorded at: the PSD summed over the accelerometer axes and integrated up to max_freq.
using VibrationsEnergy = std::map<double, std::map<double, double>>;

// Energy of every vibrations recording in `measurements`. Recordings named otherwise are ignored,
// and recordings too short to analyse are counted in `unusable`.
VibrationsEnergy vibrations_energy(const std::vector<Measurement> &measurements, double max_freq, size_t *unusable = nullptr);

// The analysis proper, from the per-recording energies.
std::optional<VibrationsProfile> analyze_vibrations_energy(const VibrationsEnergy &energy, std::string &error);

// vibrations_energy() followed by analyze_vibrations_energy().
std::optional<VibrationsProfile> analyze_vibrations(const std::vector<Measurement> &measurements, double max_freq,
                                                    std::string &error);

} // namespace Slic3r::ShakeTune
