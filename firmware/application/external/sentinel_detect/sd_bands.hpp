/*
 * sd_bands.hpp — Sentinel Detect band plan + FPV channel tables (portable, header-only).
 *
 * GPLv2-or-later. Part of the Sentinel Detect PortaPack external app, a derivative work of
 * the PortaPack-Mayhem firmware (GPLv2+) and an on-device port of the SentinelRF Detector
 * (drone-sentinel). This header is dependency-free so it compiles both in the Mayhem
 * firmware and in a host unit test.
 *
 * Faithful C++ port of drone-sentinel/coordinator/sentinel/sdr/plan.py:
 *   BandScan table (SCAN_PLANS["all"] and friends), FPV58_BANDS / FPV12_CHANNELS,
 *   fpv58_label / fpv12_label. Frequencies in MHz throughout.
 */
#ifndef SENTINEL_DETECT_SD_BANDS_HPP
#define SENTINEL_DETECT_SD_BANDS_HPP

#include <cstdint>
#include <cmath>

namespace sentinel_detect {

// Round-half-to-even, matching Python's round() (which the source uses at every rounding
// site). rintf uses the default FE_TONEAREST rounding mode = ties-to-even. Shared by the
// detection cores; do NOT use (int)(x+0.5), which is half-away-from-zero and diverges on
// exact .5 boundaries (systematically reachable on the 1 MHz / 0.5 MHz band grids).
inline int32_t sd_round_even(float x) { return (int32_t)rintf(x); }

// The five lanes a sweep can produce (models.py Lane). dji_droneid is intentionally omitted
// (it needs an operator-supplied decoder; a power sweep cannot do it).
enum class Lane : uint8_t {
    video58 = 0,
    video12 = 1,
    control_24 = 2,
    control_sub = 3,
    sdr_sweep = 4,
};

inline const char* lane_name(Lane l) {
    switch (l) {
        case Lane::video58: return "video58";
        case Lane::video12: return "video12";
        case Lane::control_24: return "control24";
        case Lane::control_sub: return "controlsub";
        default: return "sdr_sweep";
    }
}

// Short lane badge for the on-device detections list.
inline const char* lane_badge(Lane l) {
    switch (l) {
        case Lane::video58: return "VID5.8";
        case Lane::video12: return "VID1.2";
        case Lane::control_24: return "CTL2.4";
        case Lane::control_sub: return "CTLsub";
        default: return "SWEEP ";
    }
}

// Node-side lane confidences, mirroring the firmware lanes (lanes.py:42-48).
static constexpr float CONF_VIDEO_SYNC = 0.80f;      // raster confirmed by an IQ dwell
static constexpr float CONF_VIDEO58_ENERGY = 0.70f;  // energy only (5.8)
static constexpr float CONF_VIDEO12_ENERGY = 0.65f;  // energy only (1.2)
static constexpr float CONF_CONTROL_HOPPING = 0.60f; // narrow + confirmed hop pattern
static constexpr float CONF_CONTROL_PLAIN = 0.55f;   // narrow, no hop pattern yet
static constexpr float CONF_SDR_SWEEP = 0.50f;       // generic wideband energy

// A band to sweep. Mirrors plan.py:69-93 BandScan (only the fields the on-device pipeline
// uses; per-band gains do not exist — LNA/VGA/amp are global node settings).
struct BandScan {
    const char* name;
    Lane lane;
    float f_lo_mhz;
    float f_hi_mhz;
    float bin_width_khz;      // finest bin this band asks for (advisory; hardware bin may be finer)
    float margin_db;          // dB over the CFAR floor
    int32_t nominal_mhz;      // fixed band_mhz for FHSS control lanes, else -1
    bool fhss;
    float max_narrow_bw_mhz;  // control upper width bound (<=): a hit wider than this is not a control carrier
    float min_video_bw_mhz;   // video lower width bound (>=): a hit narrower than this is not video
};

// MARGIN_VIDEO_DB / MARGIN_CONTROL_DB (plan.py:111-112) — both 12 dB, deliberately not the
// ESP32 tier's 8/10 dB (a single-look periodogram bin has ~5.57 dB spread). A PortaPack
// reading an integrated FFT is closer to the firmware case, so 12 dB is a safe start and is
// operator-tunable in the UI.
static constexpr float MARGIN_DB_DEFAULT = 12.0f;

// The band table. Order within a plan is the round-robin sweep order.
// (plan.py:113-167.) control_sub US (902-928) is used for the default "all" plan;
// EU (863-870) and 433 are available via other plans.
static const BandScan BAND_VIDEO58 = {"video58", Lane::video58, 5645.0f, 5945.0f, 500.0f, MARGIN_DB_DEFAULT, -1, false, 3.0f, 2.0f};
static const BandScan BAND_VIDEO12 = {"video12", Lane::video12, 1060.0f, 1380.0f, 500.0f, MARGIN_DB_DEFAULT, -1, false, 3.0f, 2.0f};
static const BandScan BAND_CONTROL_24 = {"control_24", Lane::control_24, 2400.0f, 2483.5f, 100.0f, MARGIN_DB_DEFAULT, 2440, true, 3.0f, 2.0f};
static const BandScan BAND_CONTROL_SUB_US = {"control_sub", Lane::control_sub, 902.0f, 928.0f, 100.0f, MARGIN_DB_DEFAULT, 915, true, 2.0f, 2.0f};
static const BandScan BAND_CONTROL_SUB_EU = {"control_sub", Lane::control_sub, 863.0f, 870.0f, 100.0f, MARGIN_DB_DEFAULT, 868, true, 2.0f, 2.0f};
static const BandScan BAND_CONTROL_433 = {"control_433", Lane::control_sub, 433.05f, 434.79f, 50.0f, MARGIN_DB_DEFAULT, 433, true, 1.0f, 2.0f};
static const BandScan BAND_WIDE_24 = {"wide_24", Lane::sdr_sweep, 2400.0f, 2483.5f, 500.0f, MARGIN_DB_DEFAULT, -1, false, 3.0f, 2.0f};
static const BandScan BAND_WIDE_58 = {"wide_58", Lane::sdr_sweep, 5150.0f, 5900.0f, 1000.0f, MARGIN_DB_DEFAULT, -1, false, 3.0f, 2.0f};

// Scan plans (plan.py:170-187). Index into these with ScanPlan; a plan is a list of
// pointers into the band table above.
enum class ScanPlan : uint8_t { all = 0, all_eu = 1, fpv = 2, control = 3, wideband = 4 };

struct ScanPlanDef {
    const char* label;
    const BandScan* bands[4];
    uint8_t count;
};

static const ScanPlanDef SCAN_PLANS[] = {
    {"all", {&BAND_CONTROL_SUB_US, &BAND_VIDEO12, &BAND_CONTROL_24, &BAND_VIDEO58}, 4},
    {"all-eu", {&BAND_CONTROL_SUB_EU, &BAND_VIDEO12, &BAND_CONTROL_24, &BAND_VIDEO58}, 4},
    {"fpv", {&BAND_VIDEO12, &BAND_VIDEO58, nullptr, nullptr}, 2},
    {"control", {&BAND_CONTROL_SUB_US, &BAND_CONTROL_24, nullptr, nullptr}, 2},
    {"wideband", {&BAND_WIDE_24, &BAND_WIDE_58, nullptr, nullptr}, 2},
};
static const int SCAN_PLANS_COUNT = static_cast<int>(sizeof(SCAN_PLANS) / sizeof(SCAN_PLANS[0]));

// --- FPV channel tables (plan.py:28-37), byte-identical to the ESP32 RX5808_CH_FREQ and to
//     fpv_detect's fpv_frequencies. Used only to LABEL a video detection. -----------------
static const int32_t FPV58_CHANNELS[] = {
    // A                      B                      E                      F                      R
    5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725,
    5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866,
    5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945,
    5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880,
    5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917,
};
static const char FPV58_BAND_LABELS[] = {'A', 'B', 'E', 'F', 'R'};
static const int FPV58_CHANNELS_COUNT = 40;

static const int32_t FPV12_CHANNELS[] = {1080, 1120, 1160, 1200, 1240, 1280, 1320, 1360};
static const int FPV12_CHANNELS_COUNT = 8;

static constexpr float FPV58_LABEL_TOL_MHZ = 9.0f;
static constexpr float FPV12_LABEL_TOL_MHZ = 20.0f;

// fpv58_label: label + snapped band_mhz for a 5.8 GHz video carrier midpoint.
// Two Python behaviours are reproduced separately (they differ exactly at tol):
//   - band_mhz snap (lanes.py:138): nearest channel if |Δ| <= tol (INCLUSIVE) else round(mid)
//   - label       (plan.py:40-54): "F4"-style only if |Δ| <  tol (STRICT), else empty
inline void fpv58_label(float mhz, char* out, int32_t* snapped_out) {
    int best = -1;
    float best_d = 1e9f;
    for (int i = 0; i < FPV58_CHANNELS_COUNT; ++i) {
        float d = std::fabs(static_cast<float>(FPV58_CHANNELS[i]) - mhz);
        if (d < best_d) { best_d = d; best = i; }
    }
    if (snapped_out)
        *snapped_out = (best >= 0 && best_d <= FPV58_LABEL_TOL_MHZ) ? FPV58_CHANNELS[best]
                                                                    : sd_round_even(mhz);
    if (best >= 0 && best_d < FPV58_LABEL_TOL_MHZ) {
        out[0] = FPV58_BAND_LABELS[best / 8];
        out[1] = static_cast<char>('1' + (best % 8));
        out[2] = '\0';
    } else {
        out[0] = '\0';
    }
}

// fpv12_label: label + snapped band_mhz for a 1.2 GHz video carrier midpoint (same rules).
inline void fpv12_label(float mhz, char* out, int32_t* snapped_out) {
    int best = -1;
    float best_d = 1e9f;
    for (int i = 0; i < FPV12_CHANNELS_COUNT; ++i) {
        float d = std::fabs(static_cast<float>(FPV12_CHANNELS[i]) - mhz);
        if (d < best_d) { best_d = d; best = i; }
    }
    if (snapped_out)
        *snapped_out = (best >= 0 && best_d <= FPV12_LABEL_TOL_MHZ) ? FPV12_CHANNELS[best]
                                                                    : sd_round_even(mhz);
    if (best >= 0 && best_d < FPV12_LABEL_TOL_MHZ) {
        out[0] = 'C'; out[1] = 'H'; out[2] = static_cast<char>('1' + best); out[3] = '\0';
    } else {
        out[0] = '\0';
    }
}

}  // namespace sentinel_detect

#endif  // SENTINEL_DETECT_SD_BANDS_HPP
