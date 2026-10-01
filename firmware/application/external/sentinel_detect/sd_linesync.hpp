/*
 * sd_linesync.hpp — analog-video line-sync raster confirmation (portable, header-only).
 *
 * GPLv2-or-later. Faithful C++ port of drone-sentinel/coordinator/sentinel/sdr/linesync.py
 * (goertzel_power / MIN_FS_HZ / line rates) and linefit.py (search_line_rate over a band of
 * line-rate hypotheses). The Python is itself a port of the ESP32 firmware's linesync.h, so
 * this closes the loop back to firmware.
 *
 * PHASE 2: this is the discriminator that separates analog FPV from digital/WiFi. It runs on
 * an FM-demodulated baseband block (instantaneous frequency, real samples). Wiring the IQ/FM
 * dwell on-device (a baseband swap to WFM audio, capture, demod) is the remaining Phase-2
 * work; this math is ready and host-testable now. v1 ships video lanes energy-only.
 */
#ifndef SENTINEL_DETECT_SD_LINESYNC_HPP
#define SENTINEL_DETECT_SD_LINESYNC_HPP

#include <cmath>

namespace sentinel_detect {

// Horizontal line rates (Hz). Must match linesync.h exactly.
static constexpr float LINE_NTSC_HZ = 15734.264f;   // NTSC/EIA-170 = 15750/1.001
static constexpr float LINE_PAL_HZ = 15625.0f;      // PAL/CCIR exact
static constexpr float NTSC_PAL_SEP_HZ = LINE_NTSC_HZ - LINE_PAL_HZ;  // 109.264

static constexpr float SD_MIN_FS_HZ = 2.2f * LINE_NTSC_HZ;  // ~34.6 kHz
static constexpr int SD_MIN_SAMPLES = 64;

// Line-rate hypothesis band (linefit.py:40-41).
static constexpr float SEARCH_LO_HZ = 15400.0f;
static constexpr float SEARCH_HI_HZ = 15900.0f;
static constexpr float MIN_SEP_BINS = 3.0f;

static constexpr float SYNC_RATIO_DEFAULT = 0.08f;  // VIDEO58_SYNC_RATIO

enum class VideoStd : uint8_t { none = 0, analog = 1, ntsc = 2, pal = 3 };

inline const char* video_std_name(VideoStd s) {
    switch (s) {
        case VideoStd::ntsc: return "NTSC";
        case VideoStd::pal: return "PAL";
        case VideoStd::analog: return "analog";
        default: return "none";
    }
}

struct LineRateResult {
    bool present;
    float ratio;              // line-rate energy fraction at the best hypothesis
    float line_hz;            // best-fit line rate
    VideoStd standard;
    bool standard_confident;  // false => cannot resolve NTSC from PAL
    float bin_hz;             // baseband resolution 1/T
    float sep_bins;           // NTSC/PAL separation in bins
    int n_hypotheses;
};

// Goertzel power at f_hz over DC-removed real samples x[0..n). Identical recurrence to
// linesync::goertzel_power. Not normalized. (Operates on already-DC-removed input.)
inline double sd_goertzel_power(const float* x, int n, float f_hz, float fs_hz) {
    double coeff = 2.0 * std::cos(2.0 * M_PI * (double)f_hz / (double)fs_hz);
    double s1 = 0.0, s2 = 0.0;
    for (int i = 0; i < n; ++i) {
        double s0 = (double)x[i] + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

// Goertzel over DC-removed samples without a scratch buffer (subtracts `mean` per sample).
inline double goertzel_dc(const float* samples, int n, float mean, float f_hz, float fs_hz) {
    double coeff = 2.0 * std::cos(2.0 * M_PI * (double)f_hz / (double)fs_hz);
    double s1 = 0.0, s2 = 0.0;
    for (int i = 0; i < n; ++i) {
        double s0 = ((double)samples[i] - (double)mean) + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

// search_line_rate (linefit.py:63-114). `samples` is a real baseband block (FM-demodulated
// instantaneous frequency). Scans [f_lo,f_hi] at bin/4 steps; ratio = Goertzel/(energy*n/2).
inline LineRateResult search_line_rate(const float* samples, int n, float fs_hz,
                                       float ratio_thresh = SYNC_RATIO_DEFAULT,
                                       float f_lo = SEARCH_LO_HZ, float f_hi = SEARCH_HI_HZ,
                                       float min_sep_bins = MIN_SEP_BINS) {
    LineRateResult r{false, 0.0f, 0.0f, VideoStd::none, false, 0.0f, 0.0f, 0};
    if (n < SD_MIN_SAMPLES || fs_hz < SD_MIN_FS_HZ) return r;

    // DC removal.
    double sum = 0.0;
    for (int i = 0; i < n; ++i) sum += samples[i];
    double mean = sum / n;
    // Build a mutable DC-removed copy on the caller's buffer is not possible (const); use a
    // running approach: Goertzel needs the DC-removed samples, so subtract inline via a
    // small helper. To avoid a second buffer we compute energy here and let the Goertzel
    // subtract the mean per-sample.
    double energy = 0.0;
    for (int i = 0; i < n; ++i) {
        double v = (double)samples[i] - mean;
        energy += v * v;
    }
    if (energy <= 1e-6) return r;

    // Compute the scan grid in double to match linefit.py exactly (float accumulation of
    // the loop variable would shift the f_hi boundary and change n_hypotheses by ±1).
    double duration_s = (double)n / (double)fs_hz;
    double bin_hz = 1.0 / duration_s;
    double sep_bins = (bin_hz > 0.0) ? ((double)NTSC_PAL_SEP_HZ / bin_hz) : 0.0;
    double step = bin_hz / 4.0;
    if (step < 0.5) step = 0.5;
    double norm = energy * (double)n * 0.5;

    double best_ratio = -1.0;
    double best_f = 0.0;
    int count = 0;
    for (double f = (double)f_lo; f <= (double)f_hi; f += step) {
        double ratio = goertzel_dc(samples, n, (float)mean, (float)f, fs_hz) / norm;
        if (ratio > best_ratio) { best_ratio = ratio; best_f = f; }
        ++count;
    }

    bool present = best_ratio >= (double)ratio_thresh;
    bool confident = sep_bins >= min_sep_bins;
    VideoStd standard;
    if (!present)
        standard = VideoStd::none;
    else if (!confident)
        standard = VideoStd::analog;
    else
        standard = (std::fabs(best_f - (double)LINE_NTSC_HZ) <= std::fabs(best_f - (double)LINE_PAL_HZ))
                       ? VideoStd::ntsc : VideoStd::pal;

    r.present = present;
    r.ratio = (float)best_ratio;
    r.line_hz = (float)best_f;
    r.standard = standard;
    r.standard_confident = present && confident;
    r.bin_hz = (float)bin_hz;
    r.sep_bins = (float)sep_bins;
    r.n_hypotheses = count;
    return r;
}

}  // namespace sentinel_detect

#endif  // SENTINEL_DETECT_SD_LINESYNC_HPP
