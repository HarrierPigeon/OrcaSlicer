// Input shaper smoothing and acceleration limits, translated from Klipper (Copyright (C) 2020-2026
// Dmitry Butyugin, GNU GPLv3). See InputShaperLimits.cpp for the sources and their notices.

#pragma once

#include <optional>
#include <string>
#include <vector>

namespace Slic3r::ShakeTune {

// An input shaper as Klipper defines it: impulse amplitudes A at times T (seconds).
struct InputShaper
{
    std::vector<double> A;
    std::vector<double> T;
};

// Klipper's shaper definitions (klippy/extras/shaper_defs.py): zv, mzv, zvd, ei, 2hump_ei, 3hump_ei.
// Returns nullopt for any other name (Kalico's smooth_* shapers, a parametrised "mzv(n=4)").
std::optional<InputShaper> make_input_shaper(const std::string &name, double freq, double damping_ratio);

// How far, in mm, the shaper rounds a corner taken at `accel` with square corner velocity `scv`
// (Klipper's ShaperCalibrate._get_shaper_smoothing).
double input_shaper_smoothing(const InputShaper &shaper, double accel, double scv);

// The highest acceleration that keeps the shaper's smoothing within Klipper's 0.12 mm target
// (ShaperCalibrate.find_shaper_max_accel). This is the "max_accel <= N" figure Klipper and
// Shake&Tune print next to each shaper recommendation.
double input_shaper_max_accel(const InputShaper &shaper, double scv);

// Klipper rounds the suggestion to the nearest 100 mm/s² before printing it.
inline double round_max_accel(double accel) { return double(long(accel / 100. + 0.5)) * 100.; }

} // namespace Slic3r::ShakeTune
