/*
 * sd_hop.hpp — frequency-hopping control-link detector (portable, header-only).
 *
 * GPLv2-or-later. Faithful C++ port of drone-sentinel/coordinator/sentinel/sdr/hop.py
 * (HopTracker / HopState). One tracker per FHSS band. The discriminator: >=hop_min distinct
 * 1 MHz channels hot within a `ttl_frames` window, reported only when a 3-of-6 N-of-M
 * debounce over "any narrow in-band hit this frame" fires on a frame that is itself active.
 *
 * Mirrors the ESP32 control lanes' HOP_TTL=3 / 3-of-6 debounce. Unlike the firmware, the
 * CFAR floor removes the cold-start transient, so there is no −128 dBm floor init here.
 */
#ifndef SENTINEL_DETECT_SD_HOP_HPP
#define SENTINEL_DETECT_SD_HOP_HPP

#include <cstdint>
#include "sd_bands.hpp"
#include "sd_cfar.hpp"

namespace sentinel_detect {

struct HopState {
    int hops = 0;             // distinct live 1 MHz channels
    bool hopping = false;     // hops >= hop_min
    bool active_now = false;  // any narrow in-band hit this frame
    bool fired = false;       // debounce satisfied AND active_now → emit a control message
    float peak_dbm = -200.0f; // strongest narrow hit's power (valid iff active_now/has_peak)
    bool has_peak = false;
    float peak_channel_mhz = 0.0f;
    int frames_observed = 0;
    float hop_rate_hz = -1.0f;  // order-of-magnitude estimate; <0 = unknown
};

inline int sd_popcount16(uint16_t v) {
    int c = 0;
    while (v) { v &= (uint16_t)(v - 1); ++c; }
    return c;
}

class HopTracker {
   public:
    // Live 1 MHz channels tracked at once. control_24 spans ~83 channels, but with TTL=3
    // only a handful are ever hot simultaneously; 64 is ample and saves app RAM.
    static constexpr int MAX_CHANNELS = 64;

    float channel_khz = 1000.0f;
    int ttl_frames = 3;
    int hop_min = 3;
    int debounce_n = 3;
    int debounce_m = 6;

    const BandScan* band = nullptr;

    void init(const BandScan* b) {
        band = b;
        window_ = 0;
        frames_ = 0;
        first_t_ = -1.0f;
        last_t_ = 0.0f;
        n_ch_ = 0;
    }

    // Feed one frame's surviving hits. Returns the HopState for this frame.
    HopState update(const SpectrumHit* hits, int nhits, float t) {
        ++frames_;
        if (first_t_ < 0.0f) first_t_ = t;
        last_t_ = t;

        // Age every live channel, then refresh channels seen this frame (order matters:
        // a channel seen now survives exactly ttl_frames frames including this one).
        for (int i = 0; i < n_ch_;) {
            if (--ttl_[i] <= 0) {
                // erase i by swapping last in
                ch_[i] = ch_[n_ch_ - 1];
                ttl_[i] = ttl_[n_ch_ - 1];
                --n_ch_;
            } else {
                ++i;
            }
        }

        bool active_now = false;
        bool has_peak = false;
        float peak_dbm = -200.0f;
        float peak_mhz = 0.0f;

        for (int i = 0; i < nhits; ++i) {
            const SpectrumHit& h = hits[i];
            // admit only in-band, narrow hits
            if (h.center_mhz < band->f_lo_mhz || h.center_mhz > band->f_hi_mhz) continue;
            if (h.bw_mhz() > band->max_narrow_bw_mhz) continue;
            active_now = true;
            int ch = channel_of(h.center_mhz);
            refresh(ch);
            if (!has_peak || h.peak_dbm > peak_dbm) {
                has_peak = true;
                peak_dbm = h.peak_dbm;
                peak_mhz = h.center_mhz;
            }
        }

        window_ = (uint16_t)(((window_ << 1) | (active_now ? 1u : 0u)) & 0xFFFFu);
        uint16_t mask = (uint16_t)((1u << debounce_m) - 1u);
        int in_window = sd_popcount16((uint16_t)(window_ & mask));
        bool fired = in_window >= debounce_n;

        HopState st;
        st.hops = n_ch_;
        st.hopping = n_ch_ >= hop_min;
        st.active_now = active_now;
        st.fired = fired && active_now;
        st.has_peak = has_peak;
        st.peak_dbm = peak_dbm;
        st.peak_channel_mhz = peak_mhz;
        st.frames_observed = frames_;
        st.hop_rate_hz = hop_rate();
        return st;
    }

    // After emitting a control message: zero ONLY the debounce window (channels/TTL survive,
    // so the next report needs 3 fresh active frames but keeps its channel evidence).
    void reset_debounce() { window_ = 0; }

   private:
    uint16_t window_ = 0;
    int frames_ = 0;
    float first_t_ = -1.0f;
    float last_t_ = 0.0f;

    int ch_[MAX_CHANNELS] = {};
    int ttl_[MAX_CHANNELS] = {};
    int n_ch_ = 0;

    int channel_of(float mhz) const {
        // int(round(mhz*1000/channel_khz)) — banker's rounding to match Python round().
        return sd_round_even(mhz * 1000.0f / channel_khz);
    }

    void refresh(int ch) {
        for (int i = 0; i < n_ch_; ++i) {
            if (ch_[i] == ch) { ttl_[i] = ttl_frames; return; }
        }
        if (n_ch_ < MAX_CHANNELS) {
            ch_[n_ch_] = ch;
            ttl_[n_ch_] = ttl_frames;
            ++n_ch_;
        }
    }

    float hop_rate() const {
        if (!(n_ch_ > 1 && frames_ > 1)) return -1.0f;
        float elapsed = last_t_ - first_t_;
        if (!(elapsed > 0.0f)) return -1.0f;
        float per_frame = elapsed / (float)(frames_ - 1 > 1 ? frames_ - 1 : 1);
        float denom = per_frame * (float)ttl_frames;
        if (denom < 1e-6f) denom = 1e-6f;
        float r = (float)n_ch_ / denom;
        return roundf(r * 100.0f) / 100.0f;  // round(rate, 2) — matches hop.py:120
    }
};

}  // namespace sentinel_detect

#endif  // SENTINEL_DETECT_SD_HOP_HPP
