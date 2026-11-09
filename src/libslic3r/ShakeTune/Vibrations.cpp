// Shake&Tune machine vibrations analysis.
//
// The analysis is translated from Shake&Tune (https://github.com/Frix-x/klippain-shaketune, main
// at 354ee4b): shaketune/graph_creators/computations/vibrations_computation.py and
// shaketune/helpers/common_func.py, which carry this notice:
//     Shake&Tune: 3D printer analysis tools
//     Copyright (C) 2024 Félix Boisselier <felix@fboisselier.fr> (Frix_x on Discord)
//     Licensed under the GNU General Public License v3.0 (GPL-3.0)
// The power spectral density it relies on is translated from Klipper's
// klippy/extras/shaper_calibrate.py, which carries this notice:
//     Copyright (C) 2020-2024  Dmitry Butyugin <dmbutyugin@google.com>
//     This file may be distributed under the terms of the GNU GPLv3 license.
// Function names follow the Python originals so the two can be compared side by side; deviations
// from them are marked "Differs from Shake&Tune". The per-motor curves, peaks and bands at the end
// are this file's own addition.

#include "Vibrations.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <iterator>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>
#include <unsupported/Eigen/FFT>

// zstd's streaming decoder, linked from the copy of zstd that c-blosc bundles (blosc is already a
// dependency through OpenVDB). Only the decades-stable streaming API is declared, as in zstd.h.
extern "C" {
typedef struct ZSTD_DCtx_s ZSTD_DStream;
typedef struct ZSTD_inBuffer_s { const void *src; size_t size; size_t pos; } ZSTD_inBuffer;
typedef struct ZSTD_outBuffer_s { void *dst; size_t size; size_t pos; } ZSTD_outBuffer;
ZSTD_DStream *ZSTD_createDStream(void);
size_t        ZSTD_freeDStream(ZSTD_DStream *zds);
size_t        ZSTD_initDStream(ZSTD_DStream *zds);
size_t        ZSTD_decompressStream(ZSTD_DStream *zds, ZSTD_outBuffer *output, ZSTD_inBuffer *input);
size_t        ZSTD_DStreamOutSize(void);
unsigned      ZSTD_isError(size_t code);
const char   *ZSTD_getErrorName(size_t code);
}

namespace Slic3r::ShakeTune {

namespace {

constexpr double PI = 3.14159265358979323846;

// --- Klipper shaper_calibrate.py -----------------------------------------------------------------

constexpr double WINDOW_T_SEC = 0.5;

// Modified Bessel function of the first kind, order 0 (numpy.i0), by its power series.
double bessel_i0(double x)
{
    double sum = 1., term = 1.;
    const double q = x * x / 4.;
    for (int k = 1; k < 500; ++k) {
        term *= q / (double(k) * double(k));
        sum += term;
        if (term < sum * 1e-17)
            break;
    }
    return sum;
}

// numpy.kaiser(M, beta)
std::vector<double> kaiser_window(size_t M, double beta)
{
    std::vector<double> w(M, 1.);
    if (M < 2)
        return w;
    const double alpha = double(M - 1) / 2.;
    const double denom = bessel_i0(beta);
    for (size_t n = 0; n < M; ++n) {
        const double r = (double(n) - alpha) / alpha;
        w[n]           = bessel_i0(beta * std::sqrt(std::max(0., 1. - r * r))) / denom;
    }
    return w;
}

// ShaperCalibrate._psd: Welch's method over windows of nfft with 50% overlap, each window
// detrended to its mean, one-sided spectrum. Adds the result into `psd_out`.
void accumulate_psd(const std::vector<double> &x, double fs, size_t nfft, const std::vector<double> &window,
                    Eigen::FFT<double> &fft, std::vector<double> &psd_out)
{
    double win_sq = 0.;
    for (double v : window)
        win_sq += v * v;
    const double scale   = 1. / win_sq;
    const size_t overlap = nfft / 2;
    const size_t step    = nfft - overlap;
    const size_t windows = (x.size() - overlap) / step;
    const size_t bins    = nfft / 2 + 1;

    std::vector<double>               psd(bins, 0.);
    std::vector<double>               seg(nfft);
    std::vector<std::complex<double>> spec;
    for (size_t w = 0; w < windows; ++w) {
        const double *src  = x.data() + w * step;
        const double  mean = std::accumulate(src, src + nfft, 0.) / double(nfft);
        for (size_t i = 0; i < nfft; ++i)
            seg[i] = window[i] * (src[i] - mean);
        fft.fwd(spec, seg);
        for (size_t k = 0; k < bins; ++k)
            psd[k] += std::norm(spec[k]);
    }
    for (size_t k = 0; k < bins; ++k) {
        double v = psd[k] * scale / fs / double(windows);
        // One-sided spectrum: double everything but DC and the unpaired Nyquist bin.
        if (k != 0 && k != bins - 1)
            v *= 2.;
        psd_out[k] += v;
    }
}

// --- Shake&Tune helpers/common_func.py -----------------------------------------------------------

// numpy.trapz(y, x)
double trapz(const double *y, const double *x, size_t n)
{
    double s = 0.;
    for (size_t i = 1; i < n; ++i)
        s += 0.5 * (y[i] + y[i - 1]) * (x[i] - x[i - 1]);
    return s;
}

// numpy.trapz(y) with unit spacing
double trapz_unit(const double *y, size_t n)
{
    double s = 0.;
    for (size_t i = 1; i < n; ++i)
        s += 0.5 * (y[i] + y[i - 1]);
    return s;
}

double max_of(const std::vector<double> &v) { return v.empty() ? 0. : *std::max_element(v.begin(), v.end()); }

// detect_peaks(): local maxima of a moving-average smoothed curve above an absolute threshold that
// also stand out from their vicinity by a relative height, refined to the raw curve's maximum.
// Returns indices into `data`.
std::vector<size_t> detect_peaks(const std::vector<double> &data, double detection_threshold,
                                 double relative_height_threshold, size_t window_size, size_t vicinity)
{
    const size_t n = data.size();
    if (n < window_size + 2)
        return {};
    // np.convolve(data, ones(w)/w, 'valid'), front-padded with window_size // 2 copies of the
    // mean of the first window.
    std::vector<double> smoothed;
    const double        head_mean = std::accumulate(data.begin(), data.begin() + window_size, 0.) / double(window_size);
    smoothed.assign(window_size / 2, head_mean);
    for (size_t i = 0; i + window_size <= n; ++i)
        smoothed.push_back(std::accumulate(data.begin() + i, data.begin() + i + window_size, 0.) / double(window_size));

    const size_t        ns = smoothed.size();
    std::vector<size_t> peaks;
    for (size_t p = 1; p + 1 < ns; ++p) {
        if (!(smoothed[p - 1] < smoothed[p] && smoothed[p] > smoothed[p + 1]) || !(smoothed[p] > detection_threshold))
            continue;
        const size_t lo = p >= vicinity ? p - vicinity : 0;
        const size_t hi = std::min(ns, p + vicinity + 1);
        const double local_min = *std::min_element(smoothed.begin() + lo, smoothed.begin() + hi);
        if (smoothed[p] - local_min > relative_height_threshold * smoothed[p])
            peaks.push_back(p);
    }

    std::vector<size_t> refined;
    for (size_t p : peaks) {
        const size_t lo = p >= vicinity ? p - vicinity : 0;
        const size_t hi = std::min(n, p + vicinity + 1);
        // Differs from Shake&Tune: it offsets the argmax by `p - vicinity` even when the slice was
        // clipped at the start of the curve, which lands on a wrong (even negative) index for a
        // peak in the first `vicinity` samples. Offset by where the slice really starts.
        refined.push_back(lo + size_t(std::max_element(data.begin() + lo, data.begin() + hi) - (data.begin() + lo)));
    }
    return refined;
}

struct IndexRange
{
    size_t start;
    size_t end;
    double mean_pct;
};

// identify_low_energy_zones(): stretches below "mean + range/4 - threshold * std", with their mean
// level as a percentage of the curve's maximum, quietest first.
std::vector<IndexRange> identify_low_energy_zones(const std::vector<double> &power, double detection_threshold)
{
    const size_t n = power.size();
    if (n == 0)
        return {};
    const auto   [mn_it, mx_it] = std::minmax_element(power.begin(), power.end());
    const double mean           = std::accumulate(power.begin(), power.end(), 0.) / double(n);
    double       var            = 0.;
    for (double v : power)
        var += (v - mean) * (v - mean);
    const double std_energy = std::sqrt(var / double(n));
    const double threshold  = mean + (*mx_it - *mn_it) / 4. - detection_threshold * std_energy;

    std::vector<std::pair<size_t, size_t>> valleys;
    bool                                   in_valley = false;
    size_t                                 start     = 0;
    for (size_t i = 0; i < n; ++i) {
        if (!in_valley && power[i] < threshold) {
            in_valley = true;
            start     = i;
        } else if (in_valley && power[i] >= threshold) {
            in_valley = false;
            valleys.emplace_back(start, i);
        }
    }
    if (in_valley)
        valleys.emplace_back(start, n - 1);

    std::vector<IndexRange> out;
    const double            max_signal = *mx_it;
    for (auto [s, e] : valleys) {
        if (e <= s) // numpy's mean of an empty slice is NaN, which Shake&Tune skips
            continue;
        const double m = std::accumulate(power.begin() + s, power.begin() + e, 0.) / double(e - s);
        out.push_back({ s, e, m / max_signal * 100. });
    }
    std::stable_sort(out.begin(), out.end(), [](const IndexRange &a, const IndexRange &b) { return a.mean_pct < b.mean_pct; });
    return out;
}

// --- Shake&Tune vibrations_computation.py --------------------------------------------------------

constexpr double PEAKS_DETECTION_THRESHOLD          = 0.05;
constexpr double PEAKS_RELATIVE_HEIGHT_THRESHOLD    = 0.04;
constexpr double SPEEDS_VALLEY_DETECTION_THRESHOLD  = 0.7;
constexpr double SPEEDS_AROUND_PEAK_DELETION        = 3.; // mm/s removed around a peak
constexpr double ANGLES_VALLEY_DETECTION_THRESHOLD  = 1.1;
constexpr size_t SPECTRUM_ANGLES                    = 720;
constexpr size_t SPEED_OVERSAMPLING                 = 6;

// np.linspace(a, b, n)
std::vector<double> linspace(double a, double b, size_t n)
{
    std::vector<double> v(n);
    if (n == 1) {
        v[0] = a;
        return v;
    }
    const double step = (b - a) / double(n - 1);
    for (size_t i = 0; i < n; ++i)
        v[i] = a + step * double(i);
    v.back() = b;
    return v;
}

// _compute_speed_powers' pad_and_smooth(): a centred moving average with edge padding.
std::vector<double> pad_and_smooth(const std::vector<double> &data, size_t window = 15)
{
    const size_t half = window / 2;
    const size_t n    = data.size();
    std::vector<double> out(n);
    for (size_t i = 0; i < n; ++i) {
        double s = 0.;
        for (size_t k = 0; k < window; ++k) {
            const long j = long(i) + long(k) - long(half);
            s += data[size_t(std::clamp(j, 0L, long(n) - 1))];
        }
        out[i] = s / double(window);
    }
    return out;
}

// get_interpolated_vibrations(): linear interpolation of a per-speed curve, extrapolating the end
// segments (the projection asks for motor speeds below the slowest one measured).
double interpolate_on_speeds(const std::vector<double> &speeds, const std::vector<double> &values, double speed)
{
    const size_t n   = speeds.size();
    size_t       idx = size_t(std::lower_bound(speeds.begin(), speeds.end(), speed) - speeds.begin());
    idx              = std::clamp<size_t>(idx, 1, n - 1);
    const double lo = speeds[idx - 1], hi = speeds[idx];
    return values[idx - 1] + (speed - lo) * (values[idx] - values[idx - 1]) / (hi - lo);
}

// _filter_and_split_ranges(): cut `deletion_range` samples either side of each peak out of the
// good ranges, then merge what overlaps. Output is ordered by start.
std::vector<IndexRange> filter_and_split_ranges(const std::vector<IndexRange> &good, const std::set<size_t> &peaks,
                                                size_t deletion_range)
{
    std::vector<IndexRange> filtered;
    for (const IndexRange &r : good) {
        std::vector<size_t> inside;
        for (size_t p : peaks)
            if (p >= r.start && p <= r.end)
                inside.push_back(p);
        if (inside.empty()) {
            filtered.push_back(r);
            continue;
        }
        size_t current = r.start;
        for (size_t p : inside) {
            const size_t before_peak_end = std::max(current, p >= deletion_range ? p - deletion_range : 0);
            if (current < before_peak_end)
                filtered.push_back({ current, before_peak_end, r.mean_pct });
            current = p + deletion_range + 1;
        }
        if (current < r.end)
            filtered.push_back({ current, r.end, r.mean_pct });
    }
    // Differs from Shake&Tune: it indexes the first range unconditionally and fails when every
    // good range was cut away.
    if (filtered.empty())
        return filtered;
    std::sort(filtered.begin(), filtered.end(), [](const IndexRange &a, const IndexRange &b) { return a.start < b.start; });
    std::vector<IndexRange> merged{ filtered.front() };
    for (size_t i = 1; i < filtered.size(); ++i) {
        IndexRange &last = merged.back();
        if (filtered[i].start <= last.end) {
            last.end      = std::max(last.end, filtered[i].end);
            last.mean_pct = std::min(last.mean_pct, filtered[i].mean_pct);
        } else
            merged.push_back(filtered[i]);
    }
    return merged;
}

// Pearson correlation (np.corrcoef(a, b)[0, 1]).
double correlation(const std::vector<double> &a, const std::vector<double> &b)
{
    const size_t n  = a.size();
    const double ma = std::accumulate(a.begin(), a.end(), 0.) / double(n);
    const double mb = std::accumulate(b.begin(), b.end(), 0.) / double(n);
    double       sab = 0., saa = 0., sbb = 0.;
    for (size_t i = 0; i < n; ++i) {
        sab += (a[i] - ma) * (b[i] - mb);
        saa += (a[i] - ma) * (a[i] - ma);
        sbb += (b[i] - mb) * (b[i] - mb);
    }
    return (saa > 0. && sbb > 0.) ? sab / std::sqrt(saa * sbb) : 0.;
}

// --- Per-motor resonance bands (not in Shake&Tune) -------------------------------------------------

// A motor peak is worth avoiding when its prominence is at least this share of the loudest energy
// either motor reaches, so a quiet motor's bumps don't count...
constexpr double MOTOR_PEAK_MIN_PROMINENCE = 0.25;
// ...and when it rises at least this far above its own surroundings (here: to twice their level),
// so a curve with no resonance at all, only a loudest point, gets no band.
constexpr double MOTOR_PEAK_MIN_RELATIVE_PROMINENCE = 0.5;

// Prominence: the peak's height above the higher of the lowest points reached on either side
// before the curve climbs above the peak again.
double peak_prominence(const std::vector<double> &curve, size_t peak)
{
    const double h = curve[peak];
    double left_min = h, right_min = h;
    for (size_t i = peak; i-- > 0 && curve[i] <= h;)
        left_min = std::min(left_min, curve[i]);
    for (size_t i = peak + 1; i < curve.size() && curve[i] <= h; ++i)
        right_min = std::min(right_min, curve[i]);
    return h - std::max(left_min, right_min);
}

// The band around a peak spans the speeds where the motor's curve stays above half the peak's
// prominence (the standard half-prominence width), and at least one measurement step either side,
// which is as narrow as the measurements can resolve.
SpeedBand band_around_peak(const std::vector<double> &speeds, const std::vector<double> &curve, size_t peak,
                           double min_half_width)
{
    const size_t n     = curve.size();
    const double level = curve[peak] - 0.5 * peak_prominence(curve, peak);

    double lo = speeds.front();
    for (size_t i = peak; i-- > 0;)
        if (curve[i] <= level) {
            const double t = (level - curve[i]) / (curve[i + 1] - curve[i]);
            lo             = speeds[i] + t * (speeds[i + 1] - speeds[i]);
            break;
        }
    double hi = speeds.back();
    for (size_t i = peak + 1; i < n; ++i)
        if (curve[i] <= level) {
            const double t = (curve[i - 1] - level) / (curve[i - 1] - curve[i]);
            hi             = speeds[i - 1] + t * (speeds[i] - speeds[i - 1]);
            break;
        }
    const double centre = speeds[peak];
    return { std::max(0., std::min(lo, centre - min_half_width)), std::max(hi, centre + min_half_width) };
}

} // namespace

bool parse_stdata(const std::string &compressed, std::vector<Measurement> &out, std::string &error)
{
    out.clear();
    std::string   text;
    ZSTD_DStream *ds = ZSTD_createDStream();
    if (ds == nullptr) {
        error = "cannot create a zstd decoder";
        return false;
    }
    ZSTD_initDStream(ds);
    std::vector<char> buffer(ZSTD_DStreamOutSize());
    ZSTD_inBuffer     in{ compressed.data(), compressed.size(), 0 };
    size_t            ret = 1;
    for (;;) {
        ZSTD_outBuffer o{ buffer.data(), buffer.size(), 0 };
        ret = ZSTD_decompressStream(ds, &o, &in);
        if (ZSTD_isError(ret)) {
            error = std::string("not a Shake&Tune data file (") + ZSTD_getErrorName(ret) + ")";
            ZSTD_freeDStream(ds);
            return false;
        }
        text.append(buffer.data(), o.pos);
        // Done once all input is consumed and the decoder had no more output to give.
        if (in.pos == in.size && o.pos < o.size)
            break;
    }
    ZSTD_freeDStream(ds);
    if (ret != 0) {
        error = "the Shake&Tune data file is truncated";
        return false;
    }

    size_t line_no = 0;
    for (size_t pos = 0; pos < text.size();) {
        size_t eol = text.find('\n', pos);
        if (eol == std::string::npos)
            eol = text.size();
        ++line_no;
        const std::string_view line(text.data() + pos, eol - pos);
        pos = eol + 1;
        if (line.find_first_not_of(" \t\r") == std::string_view::npos)
            continue;
        try {
            const nlohmann::json j = nlohmann::json::parse(line.begin(), line.end());
            Measurement          m;
            m.name = j.at("name").get<std::string>();
            const nlohmann::json &samples = j.at("samples");
            m.samples.reserve(samples.size());
            for (const nlohmann::json &s : samples) {
                if (!s.is_array() || s.size() < 4)
                    throw std::runtime_error("a sample is not [time, x, y, z]");
                m.samples.push_back({ s[0].get<double>(), s[1].get<double>(), s[2].get<double>(), s[3].get<double>() });
            }
            out.push_back(std::move(m));
        } catch (const std::exception &ex) {
            error = "line " + std::to_string(line_no) + " of the Shake&Tune data file is malformed: " + ex.what();
            out.clear();
            return false;
        }
    }
    if (out.empty()) {
        error = "the Shake&Tune data file holds no measurements";
        return false;
    }
    return true;
}

bool load_stdata(const std::string &path, std::vector<Measurement> &out, std::string &error)
{
    boost::nowide::ifstream f(path, std::ios::binary);
    if (!f) {
        error = "cannot open " + path;
        return false;
    }
    const std::string bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return parse_stdata(bytes, out, error);
}

std::optional<FrequencyResponse> frequency_response(const std::vector<std::array<double, 4>> &samples)
{
    const size_t N = samples.size();
    if (N < 2)
        return std::nullopt;
    const double T = samples.back()[0] - samples.front()[0];
    if (!(T > 0.))
        return std::nullopt;
    const double fs = double(N) / T;
    // M = 1 << int(fs * WINDOW_T_SEC - 1).bit_length()
    const long long base = (long long) (fs * WINDOW_T_SEC - 1.);
    int             bits = 0;
    for (unsigned long long v = base > 0 ? (unsigned long long) base : 0ULL; v != 0; v >>= 1)
        ++bits;
    const size_t M = size_t(1) << bits;
    if (N <= M || M < 2)
        return std::nullopt;

    const std::vector<double> window = kaiser_window(M, 6.);
    Eigen::FFT<double>        fft;
    fft.SetFlag(Eigen::FFT<double>::HalfSpectrum);
    FrequencyResponse   fr;
    fr.psd_sum.assign(M / 2 + 1, 0.);
    std::vector<double> axis(N);
    for (size_t a = 1; a <= 3; ++a) {
        for (size_t i = 0; i < N; ++i)
            axis[i] = samples[i][a];
        accumulate_psd(axis, fs, M, window, fft, fr.psd_sum);
    }
    fr.freqs.resize(M / 2 + 1);
    for (size_t k = 0; k < fr.freqs.size(); ++k)
        fr.freqs[k] = double(k) * fs / double(M);
    return fr;
}

bool parse_vibrations_measurement_name(const std::string &name, double &angle, double &speed)
{
    // Shake&Tune matches r'an(\d+)_\d+sp(\d+)_\d+' and keeps only the whole parts.
    // Differs from Shake&Tune: the decimals are kept, so a fractional SPEED_INCREMENT does not make
    // two speeds collide.
    auto read = [&name](size_t &pos, double &value) {
        const size_t int_start = pos;
        while (pos < name.size() && std::isdigit((unsigned char) name[pos]))
            ++pos;
        if (pos == int_start || pos >= name.size() || name[pos] != '_')
            return false;
        const size_t frac_start = ++pos;
        while (pos < name.size() && std::isdigit((unsigned char) name[pos]))
            ++pos;
        if (pos == frac_start)
            return false;
        value = std::stod(name.substr(int_start, frac_start - 1 - int_start) + "." + name.substr(frac_start, pos - frac_start));
        return true;
    };
    for (size_t at = name.find("an"); at != std::string::npos; at = name.find("an", at + 1)) {
        size_t pos = at + 2;
        double a, s;
        if (!read(pos, a) || name.compare(pos, 2, "sp") != 0)
            continue;
        pos += 2;
        if (!read(pos, s))
            continue;
        angle = a;
        speed = s;
        return true;
    }
    return false;
}

VibrationsEnergy vibrations_energy(const std::vector<Measurement> &measurements, double max_freq, size_t *unusable)
{
    VibrationsEnergy energy;
    if (unusable)
        *unusable = 0;
    for (const Measurement &m : measurements) {
        double angle, speed;
        if (!parse_vibrations_measurement_name(m.name, angle, speed))
            continue;
        const std::optional<FrequencyResponse> fr = frequency_response(m.samples);
        if (!fr) {
            if (unusable)
                ++*unusable;
            continue;
        }
        size_t n = 0;
        while (n < fr->freqs.size() && fr->freqs[n] <= max_freq)
            ++n;
        energy[angle][speed] = trapz(fr->psd_sum.data(), fr->freqs.data(), n);
    }
    return energy;
}

std::optional<VibrationsProfile> analyze_vibrations(const std::vector<Measurement> &measurements, double max_freq,
                                                    std::string &error)
{
    size_t                 unusable = 0;
    const VibrationsEnergy energy   = vibrations_energy(measurements, max_freq, &unusable);
    if (energy.empty()) {
        error = unusable > 0 ? "the vibrations recordings are too short to analyse"
                             : "this is not a vibrations profile (CREATE_VIBRATIONS_PROFILE) recording";
        return std::nullopt;
    }
    return analyze_vibrations_energy(energy, error);
}

std::optional<VibrationsProfile> analyze_vibrations_energy(const VibrationsEnergy &energy, std::string &error)
{
    if (energy.empty()) {
        error = "there are no vibrations recordings to analyse";
        return std::nullopt;
    }
    VibrationsProfile p;
    if (energy.count(45.) && energy.count(135.)) {
        p.kinematics   = MotorKinematics::CoreXY;
        p.motor_angles = { 45., 135. };
    } else if (energy.count(0.) && energy.count(90.)) {
        p.kinematics   = MotorKinematics::Cartesian;
        p.motor_angles = { 0., 90. };
    } else {
        error = "the recordings were not taken along the motor directions (0/90 or 45/135 degrees)";
        return std::nullopt;
    }

    std::set<double> speed_set;
    for (const auto &[angle, by_speed] : energy)
        for (const auto &kv : by_speed)
            speed_set.insert(kv.first);
    p.measured_speeds.assign(speed_set.begin(), speed_set.end());
    const size_t n_measured = p.measured_speeds.size();
    if (n_measured < 2) {
        error = "the vibrations profile needs recordings at two speeds at least";
        return std::nullopt;
    }
    for (size_t m = 0; m < 2; ++m) {
        const std::map<double, double> &by_speed = energy.at(p.motor_angles[m]);
        p.motor_energy[m].resize(n_measured);
        for (size_t i = 0; i < n_measured; ++i) {
            auto it              = by_speed.find(p.measured_speeds[i]);
            p.motor_energy[m][i] = it == by_speed.end() ? 0. : it->second;
        }
    }

    // _compute_dir_speed_spectrogram(): project both motors' energy onto every direction of travel.
    p.angles = linspace(0., 360., SPECTRUM_ANGLES);
    p.speeds = linspace(p.measured_speeds.front(), p.measured_speeds.back(), n_measured * SPEED_OVERSAMPLING);
    const size_t        n_speeds = p.speeds.size();
    std::vector<double> spectrogram(SPECTRUM_ANGLES * n_speeds);
    const double        inv_sqrt2 = 1. / std::sqrt(2.);
    for (size_t a = 0; a < SPECTRUM_ANGLES; ++a) {
        const double rad = p.angles[a] * PI / 180.;
        const double c = std::cos(rad), s = std::sin(rad);
        for (size_t k = 0; k < n_speeds; ++k) {
            const double v = p.speeds[k];
            double       s1, s2;
            if (p.kinematics == MotorKinematics::CoreXY) {
                s1 = std::abs(v * (c + s) * inv_sqrt2);
                s2 = std::abs(v * (c - s) * inv_sqrt2);
            } else {
                s1 = std::abs(v * c);
                s2 = std::abs(v * s);
            }
            spectrogram[a * n_speeds + k] = interpolate_on_speeds(p.measured_speeds, p.motor_energy[0], s1) +
                                            interpolate_on_speeds(p.measured_speeds, p.motor_energy[1], s2);
        }
    }

    // _compute_speed_powers(): per speed, the worst direction and how much directions differ.
    std::vector<double> max_values(n_speeds), var_values(n_speeds);
    for (size_t k = 0; k < n_speeds; ++k) {
        double mx = -INFINITY, sum = 0.;
        for (size_t a = 0; a < SPECTRUM_ANGLES; ++a) {
            const double v = spectrogram[a * n_speeds + k];
            mx             = std::max(mx, v);
            sum += v;
        }
        const double mean = sum / double(SPECTRUM_ANGLES);
        double       var  = 0.;
        for (size_t a = 0; a < SPECTRUM_ANGLES; ++a) {
            const double d = spectrogram[a * n_speeds + k] - mean;
            var += d * d;
        }
        max_values[k] = mx;
        var_values[k] = var / double(SPECTRUM_ANGLES);
    }
    const double var_max = max_of(var_values), max_max = max_of(max_values);
    std::vector<double> metric(n_speeds);
    for (size_t k = 0; k < n_speeds; ++k) {
        const double var_scaled = var_max > 0. ? var_values[k] / var_max * max_max : 0.;
        metric[k]               = max_values[k] * var_scaled;
    }
    p.vibration_metric = pad_and_smooth(metric);
    p.max_energy       = pad_and_smooth(max_values);

    // Speeds to avoid, and the quiet ranges between them.
    const std::vector<size_t> peaks = detect_peaks(p.vibration_metric, PEAKS_DETECTION_THRESHOLD * max_of(p.vibration_metric),
                                                   PEAKS_RELATIVE_HEIGHT_THRESHOLD, 10, 10);
    for (size_t i : peaks)
        p.peak_speeds.push_back(p.speeds[i]);
    const std::vector<IndexRange> good = identify_low_energy_zones(p.vibration_metric, SPEEDS_VALLEY_DETECTION_THRESHOLD);
    const size_t deletion_range = size_t(SPEEDS_AROUND_PEAK_DELETION / (p.speeds[1] - p.speeds[0]));
    for (const IndexRange &r : filter_and_split_ranges(good, std::set<size_t>(peaks.begin(), peaks.end()), deletion_range))
        p.good_speed_ranges.push_back({ p.speeds[r.start], p.speeds[r.end], r.mean_pct });

    // _compute_angle_powers(): energy per direction, smoothed over 15 directions around the circle.
    std::vector<double> angle_powers(SPECTRUM_ANGLES);
    for (size_t a = 0; a < SPECTRUM_ANGLES; ++a)
        angle_powers[a] = trapz_unit(spectrogram.data() + a * n_speeds, n_speeds);
    p.angle_energy.resize(SPECTRUM_ANGLES);
    for (size_t a = 0; a < SPECTRUM_ANGLES; ++a) {
        double s = 0.;
        for (long d = -7; d <= 7; ++d)
            s += angle_powers[size_t((long(a) + d + long(SPECTRUM_ANGLES)) % long(SPECTRUM_ANGLES))];
        p.angle_energy[a] = s / 15.;
    }
    for (const IndexRange &r : identify_low_energy_zones(p.angle_energy, ANGLES_VALLEY_DETECTION_THRESHOLD))
        p.good_angle_ranges.push_back({ p.angles[r.start], p.angles[r.end], r.mean_pct });

    // _compute_symmetry_analysis(): correlate the two halves of the map either side of the
    // direction midway between the motors.
    {
        const size_t half         = SPECTRUM_ANGLES / 2;
        const size_t half_segment = half / 2;
        const double midpoint     = 0.5 * (p.motor_angles[0] + p.motor_angles[1]);
        const size_t split        = size_t(midpoint * (double(SPECTRUM_ANGLES) / 360.) + double(half));
        // Rows of the map extended with its last half in front, as Shake&Tune does.
        auto row = [&](size_t extended_row) { return (extended_row + SPECTRUM_ANGLES - half) % SPECTRUM_ANGLES; };
        std::vector<double> seg1, seg2;
        for (size_t r = split - half_segment; r < split; ++r)
            seg1.insert(seg1.end(), spectrogram.begin() + row(r) * n_speeds, spectrogram.begin() + (row(r) + 1) * n_speeds);
        for (size_t r = split; r < split + half_segment; ++r)
            seg2.insert(seg2.end(), spectrogram.begin() + row(r) * n_speeds, spectrogram.begin() + (row(r) + 1) * n_speeds);
        const double corr = std::max(0., correlation(seg1, seg2));
        p.symmetry_pct    = std::min(100., 100. * std::pow(corr, 0.75) + 10.);
    }

    // Per-motor curves and the bands around their peaks. Candidates are detected the way Shake&Tune
    // detects the peaks of its global profile, then kept only if they matter on the machine's scale.
    const double step = (p.measured_speeds.back() - p.measured_speeds.front()) / double(n_measured - 1);
    for (size_t m = 0; m < 2; ++m) {
        std::vector<double> &curve = p.motor_curve[m];
        curve.resize(n_speeds);
        for (size_t k = 0; k < n_speeds; ++k)
            curve[k] = interpolate_on_speeds(p.measured_speeds, p.motor_energy[m], p.speeds[k]);
    }
    const double loudest = std::max(max_of(p.motor_curve[0]), max_of(p.motor_curve[1]));
    for (size_t m = 0; m < 2; ++m) {
        const std::vector<double> &curve = p.motor_curve[m];
        for (size_t i : detect_peaks(curve, PEAKS_DETECTION_THRESHOLD * max_of(curve), PEAKS_RELATIVE_HEIGHT_THRESHOLD, 10, 10)) {
            const double prominence = peak_prominence(curve, i);
            if (prominence < MOTOR_PEAK_MIN_PROMINENCE * loudest || prominence < MOTOR_PEAK_MIN_RELATIVE_PROMINENCE * curve[i])
                continue;
            p.motor_peak_speeds[m].push_back(p.speeds[i]);
            p.motor_bands[m].push_back(band_around_peak(p.speeds, curve, i, step));
        }
        normalize_speed_bands(p.motor_bands[m]);
    }
    return p;
}

} // namespace Slic3r::ShakeTune
