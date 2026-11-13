// Input shaper smoothing and acceleration limits.
//
// Translated from Klipper's klippy/extras/shaper_defs.py and shaper_calibrate.py, which carry
// these notices:
//     Copyright (C) 2020-2026  Dmitry Butyugin <dmbutyugin@google.com>   (shaper_defs.py)
//     Copyright (C) 2020-2024  Dmitry Butyugin <dmbutyugin@google.com>   (shaper_calibrate.py)
//     This file may be distributed under the terms of the GNU GPLv3 license.
// Shake&Tune uses the same code for the "accel<=" figures on its input shaper graphs.

#include "InputShaperLimits.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace Slic3r::ShakeTune {

namespace {

constexpr double PI = 3.14159265358979323846;
// shaper_defs.SHAPER_VIBRATION_REDUCTION: the EI family is tuned to leave 1/20 of the vibration.
constexpr double SHAPER_VIBRATION_REDUCTION = 20.;
// ShaperCalibrate.find_shaper_max_accel: "empirically chosen value which produces good projections
// for max_accel without much smoothing".
constexpr double TARGET_SMOOTHING = 0.12;

InputShaper zv(double freq, double dr)
{
    const double df  = std::sqrt(1. - dr * dr);
    const double K   = std::exp(-dr * PI / df);
    const double t_d = 1. / (freq * df);
    return { { 1., K }, { 0., .5 * t_d } };
}

InputShaper zvd(double freq, double dr)
{
    const double df  = std::sqrt(1. - dr * dr);
    const double K   = std::exp(-dr * PI / df);
    const double t_d = 1. / (freq * df);
    return { { 1., 2. * K, K * K }, { 0., .5 * t_d, t_d } };
}

// The classic three-impulse MZV, which is what Klipper's parametric get_mzv_shaper() yields for its
// defaults (n=3, t=0.75) and what Kalico still defines directly.
InputShaper mzv(double freq, double dr)
{
    const double df  = std::sqrt(1. - dr * dr);
    const double K   = std::exp(-.75 * dr * PI / df);
    const double t_d = 1. / (freq * df);
    const double a1  = 1. - 1. / std::sqrt(2.);
    const double a2  = (std::sqrt(2.) - 1.) * K;
    const double a3  = a1 * K * K;
    return { { a1, a2, a3 }, { 0., .375 * t_d, .75 * t_d } };
}

InputShaper ei(double freq, double dr)
{
    const double v_tol = 1. / SHAPER_VIBRATION_REDUCTION;
    const double df    = std::sqrt(1. - dr * dr);
    const double t_d   = 1. / (freq * df);
    const double a1 = (0.24968 + 0.24961 * v_tol) + ((0.80008 + 1.23328 * v_tol) + (0.49599 + 3.17316 * v_tol) * dr) * dr;
    const double a3 = (0.25149 + 0.21474 * v_tol) + ((-0.83249 + 1.41498 * v_tol) + (0.85181 - 4.90094 * v_tol) * dr) * dr;
    const double a2 = 1. - a1 - a3;
    const double t2 = 0.4999 + (((0.46159 + 8.57843 * v_tol) * v_tol) +
                                (((4.26169 - 108.644 * v_tol) * v_tol) + ((1.75601 + 336.989 * v_tol) * v_tol) * dr) * dr) * dr;
    return { { a1, a2, a3 }, { 0., t2 * t_d, t_d } };
}

// Multi-hump EI shapers are tabulated as polynomials in the damping ratio.
template<size_t N>
InputShaper from_expansion_coeffs(double freq, double dr, const double (&t)[N][4], const double (&a)[N][4])
{
    const double tau = 1. / freq;
    InputShaper  s;
    for (size_t i = 0; i < N; ++i) {
        double u = t[i][3], v = a[i][3];
        for (int j = 2; j >= 0; --j) {
            u = u * dr + t[i][j];
            v = v * dr + a[i][j];
        }
        s.T.push_back(u * tau);
        s.A.push_back(v);
    }
    return s;
}

InputShaper hump2_ei(double freq, double dr)
{
    static const double t[4][4] = { { 0., 0., 0., 0. },
                                    { 0.49890, 0.16270, -0.54262, 6.16180 },
                                    { 0.99748, 0.18382, -1.58270, 8.17120 },
                                    { 1.49920, -0.09297, -0.28338, 1.85710 } };
    static const double a[4][4] = { { 0.16054, 0.76699, 2.26560, -1.22750 },
                                    { 0.33911, 0.45081, -2.58080, 1.73650 },
                                    { 0.34089, -0.61533, -0.68765, 0.42261 },
                                    { 0.15997, -0.60246, 1.00280, -0.93145 } };
    return from_expansion_coeffs(freq, dr, t, a);
}

InputShaper hump3_ei(double freq, double dr)
{
    static const double t[5][4] = { { 0., 0., 0., 0. },
                                    { 0.49974, 0.23834, 0.44559, 12.4720 },
                                    { 0.99849, 0.29808, -2.36460, 23.3990 },
                                    { 1.49870, 0.10306, -2.01390, 17.0320 },
                                    { 1.99960, -0.28231, 0.61536, 5.40450 } };
    static const double a[5][4] = { { 0.11275, 0.76632, 3.29160, -1.44380 },
                                    { 0.23698, 0.61164, -2.57850, 4.85220 },
                                    { 0.30008, -0.19062, -2.14560, 0.13744 },
                                    { 0.23775, -0.73297, 0.46885, -2.08650 },
                                    { 0.11244, -0.45439, 0.96382, -1.46000 } };
    return from_expansion_coeffs(freq, dr, t, a);
}

} // namespace

std::optional<InputShaper> make_input_shaper(const std::string &name, double freq, double damping_ratio)
{
    if (!(freq > 0.) || !(damping_ratio >= 0.) || damping_ratio >= 1.)
        return std::nullopt;
    if (name == "zv")       return zv(freq, damping_ratio);
    if (name == "mzv")      return mzv(freq, damping_ratio);
    if (name == "zvd")      return zvd(freq, damping_ratio);
    if (name == "ei")       return ei(freq, damping_ratio);
    if (name == "2hump_ei") return hump2_ei(freq, damping_ratio);
    if (name == "3hump_ei") return hump3_ei(freq, damping_ratio);
    return std::nullopt;
}

double input_shaper_smoothing(const InputShaper &shaper, double accel, double scv)
{
    const double half_accel = accel * .5;
    const double inv_D      = 1. / std::accumulate(shaper.A.begin(), shaper.A.end(), 0.);
    const size_t n          = shaper.T.size();
    // Time shift the shaper introduces.
    double ts = 0.;
    for (size_t i = 0; i < n; ++i)
        ts += shaper.A[i] * shaper.T[i];
    ts *= inv_D;
    // Offsets for a 90 and a 180 degree turn.
    double offset_90 = 0., offset_180 = 0.;
    for (size_t i = 0; i < n; ++i) {
        const double dt = shaper.T[i] - ts;
        if (shaper.T[i] >= ts)
            offset_90 += shaper.A[i] * (scv + half_accel * dt) * dt;
        offset_180 += shaper.A[i] * half_accel * dt * dt;
    }
    offset_90 *= inv_D * std::sqrt(2.);
    offset_180 *= inv_D;
    return std::max(offset_90, offset_180);
}

double input_shaper_max_accel(const InputShaper &shaper, double scv)
{
    // ShaperCalibrate._bisect: the largest accel for which the predicate still holds.
    auto ok = [&](double accel) { return input_shaper_smoothing(shaper, accel, scv) <= TARGET_SMOOTHING; };
    double left = 1., right = 1.;
    if (!ok(1e-9))
        return 0.;
    while (!ok(left)) {
        right = left;
        left *= .5;
    }
    if (right == left)
        while (ok(right))
            right *= 2.;
    while (right - left > 1e-8) {
        const double middle = (left + right) * .5;
        if (ok(middle))
            left = middle;
        else
            right = middle;
    }
    return left;
}

} // namespace Slic3r::ShakeTune
