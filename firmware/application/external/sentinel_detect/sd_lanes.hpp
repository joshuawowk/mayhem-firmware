/*
 * sd_lanes.hpp — lane classification + detection factories (portable, header-only).
 *
 * GPLv2-or-later. Faithful C++ port of the lane rules in
 * drone-sentinel/coordinator/sentinel/sdr/lanes.py + the dispatch in gateway.py:330-363.
 *
 * A carrier's lane is decided by the band it fell in and its width:
 *   - video band + wide (bw >= min_video_bw 2.0) → video (confirmed 0.8 / energy-only 0.7/0.65)
 *   - FHSS band + narrow (bw <= max_narrow_bw)   → control (via HopTracker; 0.6/0.55)
 *   - everything else                            → sdr_sweep (0.5)
 * band_mhz always derives from the span MIDPOINT (never the wandering peak bin).
 */
#ifndef SENTINEL_DETECT_SD_LANES_HPP
#define SENTINEL_DETECT_SD_LANES_HPP

#include <cstdint>
#include "sd_bands.hpp"
#include "sd_cfar.hpp"
#include "sd_hop.hpp"

namespace sentinel_detect {

enum class Note : uint8_t {
    none = 0,
    wide_in_control_band = 1,     // "wide carrier in a control band"
    raster_not_confirmed = 2,     // "wide carrier, analog raster not confirmed"
};

inline const char* note_text(Note n) {
    switch (n) {
        case Note::wide_in_control_band: return "wide in ctrl band";
        case Note::raster_not_confirmed: return "raster unconfirmed";
        default: return "";
    }
}

// A single detection, i.e. one wire message in the Python (models.py DetectionMessage),
// reduced to what a standalone on-device detector needs.
struct Detection {
    Lane lane;
    int32_t band_mhz;      // stable track key (snapped midpoint / fixed nominal)
    float rssi_dbm;
    float confidence;
    float bw_mhz;
    float snr_db;
    float center_mhz;      // peak bin (display only)
    char channel[4];       // video channel label e.g. "F4"/"CH4", else ""
    Note note;
    bool video_unconfirmed;  // energy-only video (no raster confirmation)
    int hops;                // control lanes: distinct channels (else 0)
    bool hopping;            // control lanes
};

// --- width predicates (lanes.py:210-218) ---
inline bool is_video_candidate(const SpectrumHit& h, const BandScan& b) {
    return (b.lane == Lane::video58 || b.lane == Lane::video12) && h.bw_mhz() >= b.min_video_bw_mhz;
}
inline bool is_control_candidate(const SpectrumHit& h, const BandScan& b) {
    return b.fhss && h.bw_mhz() <= b.max_narrow_bw_mhz;
}

// stable_band_mhz (lanes.py:88-100): midpoint on a 1 MHz grid if bw<2.0 else 5 MHz grid.
// int(round(mid/grid)*grid) with banker's rounding to match Python round().
inline int32_t stable_band_mhz(const SpectrumHit& h) {
    float mid = 0.5f * (h.low_mhz + h.high_mhz);
    float grid = (h.bw_mhz() < 2.0f) ? 1.0f : 5.0f;
    int32_t qi = sd_round_even(mid / grid);
    return qi * (int32_t)grid;
}

inline void clear_label(char* out) { out[0] = '\0'; }

// --- detection factories ---

// sdr_sweep (lanes.py:103-118): confidence 0.5, band_mhz = stable_band_mhz.
inline Detection make_sdr_sweep(const SpectrumHit& h, Note note) {
    Detection d{};
    d.lane = Lane::sdr_sweep;
    d.band_mhz = stable_band_mhz(h);
    d.rssi_dbm = h.peak_dbm;
    d.confidence = CONF_SDR_SWEEP;
    d.bw_mhz = h.bw_mhz();
    d.snr_db = h.snr_db;
    d.center_mhz = h.center_mhz;
    clear_label(d.channel);
    d.note = note;
    d.video_unconfirmed = false;
    d.hops = 0;
    d.hopping = false;
    return d;
}

// video58/video12 (lanes.py:121-172). `confirmed`==true only when a raster dwell confirmed
// the analog line rate (Phase 2). v1 calls this with confirmed=false → energy-only.
inline Detection make_video(const SpectrumHit& h, const BandScan& b, bool confirmed) {
    Detection d{};
    bool is58 = (b.lane == Lane::video58);
    d.lane = b.lane;
    float mid = 0.5f * (h.low_mhz + h.high_mhz);
    int32_t snapped = 0;
    if (is58)
        fpv58_label(mid, d.channel, &snapped);
    else
        fpv12_label(mid, d.channel, &snapped);
    d.band_mhz = snapped;  // label helpers return the snapped channel (or rounded mid)
    d.rssi_dbm = h.peak_dbm;
    if (confirmed)
        d.confidence = CONF_VIDEO_SYNC;
    else
        d.confidence = is58 ? CONF_VIDEO58_ENERGY : CONF_VIDEO12_ENERGY;
    d.bw_mhz = h.bw_mhz();
    d.snr_db = h.snr_db;
    d.center_mhz = h.center_mhz;
    d.note = Note::none;
    d.video_unconfirmed = !confirmed;
    d.hops = 0;
    d.hopping = false;
    return d;
}

// control_24/control_sub (lanes.py:175-198): band_mhz = fixed nominal so an FHSS link fuses
// into ONE track; confidence 0.6 if hopping else 0.55.
inline Detection make_control(const HopState& st, const BandScan& b) {
    Detection d{};
    d.lane = b.lane;  // control_24 or control_sub
    d.band_mhz = (b.nominal_mhz > 0) ? b.nominal_mhz : (int32_t)(b.f_lo_mhz + 0.5f);
    d.rssi_dbm = st.has_peak ? st.peak_dbm : -200.0f;
    d.confidence = st.hopping ? CONF_CONTROL_HOPPING : CONF_CONTROL_PLAIN;
    d.bw_mhz = 0.0f;
    d.snr_db = 0.0f;
    d.center_mhz = st.peak_channel_mhz;
    clear_label(d.channel);
    d.note = Note::none;
    d.video_unconfirmed = false;
    d.hops = st.hops;
    d.hopping = st.hopping;
    return d;
}

}  // namespace sentinel_detect

#endif  // SENTINEL_DETECT_SD_LANES_HPP
