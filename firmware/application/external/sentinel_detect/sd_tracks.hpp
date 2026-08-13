/*
 * sd_tracks.hpp — on-device track table (portable, header-only).
 *
 * GPLv2-or-later. Device-side replacement for the coordinator's FusionEngine
 * (drone-sentinel/coordinator/sentinel/fusion.py + models.py). The SDR pipeline emits a
 * message per hit per frame with NO debounce on video/sdr_sweep lanes (10–80/s per carrier);
 * this table dedups them into live tracks keyed on (lane, band_mhz), expires them after
 * stale_seconds (30 s), and derives an aggregate threat confidence (probabilistic OR across
 * corroborating lanes, +0.06 per extra lane, cap 0.99; alert at >= 0.6).
 */
#ifndef SENTINEL_DETECT_SD_TRACKS_HPP
#define SENTINEL_DETECT_SD_TRACKS_HPP

#include <cstdint>
#include "sd_bands.hpp"
#include "sd_lanes.hpp"

namespace sentinel_detect {

static constexpr float TRACK_STALE_S = 30.0f;
static constexpr float ALERT_CONF = 0.6f;

// LANE_BASE_CONFIDENCE (models.py:45-54) — the coordinator's per-lane trust, used for the
// probabilistic-OR fusion. (Distinct from the node CONF_* emit values.)
inline float lane_base_confidence(Lane l) {
    switch (l) {
        case Lane::video58: return 0.70f;
        case Lane::video12: return 0.65f;
        case Lane::control_24: return 0.55f;
        case Lane::control_sub: return 0.55f;
        default: return 0.50f;  // sdr_sweep
    }
}

struct Track {
    bool used = false;
    Lane lane;
    int32_t band_mhz;
    float confidence;     // last node confidence for this lane
    float last_rssi_dbm;
    float peak_rssi_dbm;
    float bw_mhz;
    float first_seen_s;
    float last_seen_s;
    uint32_t hits;
    char channel[4];
    Note note;
    bool video_unconfirmed;
    int hops;
    bool hopping;
};

class TrackTable {
   public:
    static constexpr int MAX_TRACKS = 24;

    float stale_seconds = TRACK_STALE_S;

    void reset() {
        for (int i = 0; i < MAX_TRACKS; ++i) tracks_[i].used = false;
    }

    // Insert/update a track from a detection. Returns true if a NEW track was born.
    bool upsert(const Detection& d, float now_s) {
        int s = find(d.lane, d.band_mhz);
        if (s >= 0) {
            Track& t = tracks_[s];
            t.confidence = d.confidence;
            t.last_rssi_dbm = d.rssi_dbm;
            if (d.rssi_dbm > t.peak_rssi_dbm) t.peak_rssi_dbm = d.rssi_dbm;
            t.bw_mhz = d.bw_mhz;
            t.last_seen_s = now_s;
            ++t.hits;
            copy_label(t.channel, d.channel);
            t.note = d.note;
            t.video_unconfirmed = d.video_unconfirmed;
            t.hops = d.hops;
            t.hopping = d.hopping;
            return false;
        }
        int slot = free_slot();
        if (slot < 0) slot = oldest_slot();  // reuse the stalest if full
        Track& t = tracks_[slot];
        t.used = true;
        t.lane = d.lane;
        t.band_mhz = d.band_mhz;
        t.confidence = d.confidence;
        t.last_rssi_dbm = d.rssi_dbm;
        t.peak_rssi_dbm = d.rssi_dbm;
        t.bw_mhz = d.bw_mhz;
        t.first_seen_s = now_s;
        t.last_seen_s = now_s;
        t.hits = 1;
        copy_label(t.channel, d.channel);
        t.note = d.note;
        t.video_unconfirmed = d.video_unconfirmed;
        t.hops = d.hops;
        t.hopping = d.hopping;
        return true;
    }

    // Expire stale tracks. Writes the band_mhz/lane of each expired track into `died` (up to
    // died_cap) for logging; returns the number expired.
    int expire(float now_s, Track* died, int died_cap) {
        int n = 0;
        for (int i = 0; i < MAX_TRACKS; ++i) {
            if (tracks_[i].used && (now_s - tracks_[i].last_seen_s) > stale_seconds) {
                if (died && n < died_cap) died[n] = tracks_[i];
                tracks_[i].used = false;
                ++n;
            }
        }
        return n;
    }

    int count() const {
        int n = 0;
        for (int i = 0; i < MAX_TRACKS; ++i)
            if (tracks_[i].used) ++n;
        return n;
    }

    const Track& at(int i) const { return tracks_[i]; }
    int capacity() const { return MAX_TRACKS; }

    // Aggregate threat confidence: probabilistic OR over live tracks' lane-base confidences,
    // + 0.06 per extra contributing lane, capped at 0.99. (fusion.py:117-127.)
    float aggregate_confidence() const {
        float prod = 1.0f;
        int lanes = 0;
        bool seen[5] = {false, false, false, false, false};
        for (int i = 0; i < MAX_TRACKS; ++i) {
            if (!tracks_[i].used) continue;
            int li = (int)tracks_[i].lane;
            if (li >= 0 && li < 5 && !seen[li]) {
                seen[li] = true;
                ++lanes;
                prod *= (1.0f - lane_base_confidence(tracks_[i].lane));
            }
        }
        if (lanes == 0) return 0.0f;
        float conf = 1.0f - prod + 0.06f * (float)(lanes - 1);
        if (conf > 0.99f) conf = 0.99f;
        if (conf < 0.0f) conf = 0.0f;
        return conf;
    }

    bool alerting() const { return aggregate_confidence() >= ALERT_CONF; }

   private:
    Track tracks_[MAX_TRACKS] = {};

    int find(Lane lane, int32_t band_mhz) const {
        for (int i = 0; i < MAX_TRACKS; ++i)
            if (tracks_[i].used && tracks_[i].lane == lane && tracks_[i].band_mhz == band_mhz)
                return i;
        return -1;
    }
    int free_slot() const {
        for (int i = 0; i < MAX_TRACKS; ++i)
            if (!tracks_[i].used) return i;
        return -1;
    }
    int oldest_slot() const {
        int best = 0;
        float oldest = tracks_[0].last_seen_s;
        for (int i = 1; i < MAX_TRACKS; ++i) {
            if (tracks_[i].last_seen_s < oldest) { oldest = tracks_[i].last_seen_s; best = i; }
        }
        return best;
    }
    static void copy_label(char* dst, const char* src) {
        int i = 0;
        for (; i < 3 && src[i]; ++i) dst[i] = src[i];
        dst[i] = '\0';
    }
};

}  // namespace sentinel_detect

#endif  // SENTINEL_DETECT_SD_TRACKS_HPP
