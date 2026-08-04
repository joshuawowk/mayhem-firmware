/*
 * sd_cfar.hpp — CFAR carrier detector over one sweep frame (portable, header-only).
 *
 * GPLv2-or-later. Faithful C++ port of the DEFAULT (shipped) path of
 * drone-sentinel/coordinator/sentinel/sdr/sweep.py: CfarDetector (chunked-median coarse
 * floor + hot-bin clustering into SpectrumHit) and PersistenceFilter (artifact suppression).
 *
 * The optional local_cfar / use_temporal paths in sweep.py are OFF by default there
 * (local_cfar=False, use_temporal=False) and are omitted here — the chunked coarse floor is
 * "the real mechanism". Everything else matches numerically.
 *
 * Operates on a power array already resampled to the band's native bin grid
 * (bin_width_khz), ascending in frequency, in dB. No dynamic allocation; fixed capacities
 * sized for the widest band in the plan (wide_58 = 750 MHz / 1 MHz ≈ 750 bins;
 * control_24 = 83.5 MHz / 100 kHz ≈ 835 bins).
 */
#ifndef SENTINEL_DETECT_SD_CFAR_HPP
#define SENTINEL_DETECT_SD_CFAR_HPP

#include <cstdint>
#include <cmath>

#include "sd_bands.hpp"  // sd_round_even (shared banker's-rounding helper)

namespace sentinel_detect {

static constexpr int SD_MAX_BINS = 1024;   // >= widest band's bin count
static constexpr int SD_MAX_CHUNKS = 68;   // >= SD_MAX_BINS / min_chunk(=4) capped by chunk>=... (see below)

struct SpectrumHit {
    float center_mhz;   // peak bin centre — for tuning ONLY; wanders inside a wide carrier
    float low_mhz;      // first bin centre of the run
    float high_mhz;     // last bin centre of the run
    float peak_dbm;
    float floor_dbm;
    float snr_db;       // peak - floor at the peak bin
    int n_bins;

    float bw_mhz() const {
        float w = high_mhz - low_mhz;
        return w > 0.0f ? w : 0.0f;
    }
};

// Insertion sort of a small float buffer in place (n small: chunk<=32, window<=17).
inline void sd_isort(float* a, int n) {
    for (int i = 1; i < n; ++i) {
        float v = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; --j; }
        a[j + 1] = v;
    }
}

// Median of a[0..n) (copies into scratch, sorts). n is always a chunk width (<= chunk_bins,
// which is <= 32); the small fixed buffer keeps M4 stack use trivial.
static constexpr int SD_MEDIAN_MAX = 64;
inline float sd_median(const float* a, int n) {
    if (n <= 0) return 0.0f;
    float buf[SD_MEDIAN_MAX];
    int m = n > SD_MEDIAN_MAX ? SD_MEDIAN_MAX : n;
    for (int i = 0; i < m; ++i) buf[i] = a[i];
    sd_isort(buf, m);
    int mid = m / 2;
    return (m & 1) ? buf[mid] : 0.5f * (buf[mid - 1] + buf[mid]);
}

class CfarDetector {
   public:
    float margin_db = 12.0f;
    int min_bins = 2;
    int max_gap_bins = 1;
    int chunk_bins = 32;
    int coarse_span_chunks = 8;
    float coarse_percentile = 0.25f;

    // Detect carriers in one frame. power_db[0..n) ascending in freq; bin i centre is
    // f_lo_mhz + i*bin_mhz. Writes up to max_hits SpectrumHit into out; returns the count.
    // NOT const: writes to the member scratch buffers (meds_/coarse_/win_). Those live on
    // the heap-resident detector rather than the M4 app thread's small stack — feed() runs
    // deep in the per-frame call chain (finalize_band → feed), so ~800 B of stack scratch
    // here was a primary contributor to the app-launch stack overflow.
    int feed(const float* power_db, int n, float f_lo_mhz, float bin_mhz,
             SpectrumHit* out, int max_hits) {
        if (n <= 0 || max_hits <= 0) return 0;
        if (n > SD_MAX_BINS) n = SD_MAX_BINS;

        // chunk = min(chunk_bins, max(4, n//4))  (sweep.py:284,318)
        int chunk = n / 4;
        if (chunk < 4) chunk = 4;
        if (chunk > chunk_bins) chunk = chunk_bins;

        // Per-chunk median. (meds/coarse/win alias member scratch to keep them off the stack.)
        float* meds = meds_;
        int n_chunks = 0;
        for (int start = 0; start < n; start += chunk) {
            int len = (start + chunk <= n) ? chunk : (n - start);
            if (n_chunks < SD_MAX_CHUNKS) meds[n_chunks++] = sd_median(power_db + start, len);
        }

        // Coarse floor per chunk: coarse_percentile across ±coarse_span_chunks chunk medians.
        float* coarse = coarse_;
        const int span = coarse_span_chunks;
        for (int c = 0; c < n_chunks; ++c) {
            float* win = win_;
            int wl = 0;
            int lo = c - span; if (lo < 0) lo = 0;
            int hi = c + span; if (hi > n_chunks - 1) hi = n_chunks - 1;
            for (int k = lo; k <= hi && wl < SD_WIN_MAX; ++k) win[wl++] = meds[k];
            sd_isort(win, wl);
            int idx = (int)(coarse_percentile * wl);
            if (idx > wl - 1) idx = wl - 1;
            if (idx < 0) idx = 0;
            coarse[c] = win[idx];
        }

        // Hot-bin detection + clustering (single pass; runs bridge gaps <= max_gap_bins+1).
        // run_hot counts only HOT bins (matching Python's len(run)); the span [run_first,
        // run_last] may include bridged non-hot bins but those are NOT counted.
        int count = 0;
        int run_start = -1;   // first bin index of the current run (-1 = no open run)
        int run_hot = 0;      // number of hot bins in the current run
        int last_hot = -100;

        for (int i = 0; i < n; ++i) {
            int ci = i / chunk;
            if (ci >= n_chunks) ci = n_chunks - 1;
            float floor = coarse[ci];
            bool hot = (power_db[i] - floor) >= margin_db;
            if (!hot) continue;
            if (run_start < 0) {
                run_start = i;
                run_hot = 1;
            } else if (i - last_hot <= max_gap_bins + 1) {
                ++run_hot;  // extend the current run (gap bridged); count only this hot bin
            } else {
                emit_run(power_db, coarse, chunk, n_chunks, run_start, last_hot, run_hot, f_lo_mhz, bin_mhz, out, max_hits, count);
                run_start = i;
                run_hot = 1;
            }
            last_hot = i;
        }
        if (run_start >= 0)
            emit_run(power_db, coarse, chunk, n_chunks, run_start, last_hot, run_hot, f_lo_mhz, bin_mhz, out, max_hits, count);
        return count;
    }

   private:
    // Per-frame scratch, held here (heap, via the View's cfar_ member) rather than on the
    // stack. SD_WIN_MAX bounds the ±coarse_span_chunks median window (span<=8 → <=17 used,
    // but chunk_bins can be up to 32 so keep 2*32+1 headroom).
    static constexpr int SD_WIN_MAX = 2 * 32 + 1;
    float meds_[SD_MAX_CHUNKS];
    float coarse_[SD_MAX_CHUNKS];
    float win_[SD_WIN_MAX];

    void emit_run(const float* power_db, const float* coarse, int chunk, int n_chunks,
                  int run_first, int run_last, int run_hot, float f_lo_mhz, float bin_mhz,
                  SpectrumHit* out, int max_hits, int& count) const {
        if (run_hot < min_bins || count >= max_hits) return;
        // peak bin = argmax over [run_first, run_last]
        int peak_i = run_first;
        float peak_p = power_db[run_first];
        for (int i = run_first + 1; i <= run_last; ++i) {
            if (power_db[i] > peak_p) { peak_p = power_db[i]; peak_i = i; }
        }
        int ci = peak_i / chunk; if (ci >= n_chunks) ci = n_chunks - 1;
        float floor = coarse[ci];
        SpectrumHit& h = out[count++];
        h.center_mhz = f_lo_mhz + peak_i * bin_mhz;
        h.low_mhz = f_lo_mhz + run_first * bin_mhz;
        h.high_mhz = f_lo_mhz + run_last * bin_mhz;
        h.peak_dbm = peak_p;
        h.floor_dbm = floor;
        h.snr_db = peak_p - floor;
        h.n_bins = run_hot;  // count of hot bins (matches Python len(run)), not span width
    }
};

// PersistenceFilter (sweep.py:372-434): suppress narrow (<=0.5 MHz) hits that are hot in
// >=hit_frac of the last `window` frames with level spread <=level_tol_db. Wide carriers are
// never suppressed. Keyed on the span midpoint at ~0.5 MHz resolution.
class PersistenceFilter {
   public:
    int window = 20;
    int min_observations = 8;
    float hit_frac = 0.9f;
    float level_tol_db = 6.0f;
    float max_artifact_bw_mhz = 0.5f;

    static constexpr int SLOTS = 32;  // fixed table of tracked narrow-artifact keys

    void reset() {
        for (int i = 0; i < SLOTS; ++i) { used_[i] = false; }
    }

    // Call once per frame with that frame's hits and the frame's [lo,hi] span (MHz),
    // BEFORE keep().
    void observe(const SpectrumHit* hits, int nhits, float span_lo_mhz, float span_hi_mhz) {
        // Ensure a slot exists for each hit key.
        for (int i = 0; i < nhits; ++i) ensure_slot(key_of(hits[i]));
        // One observation per key per frame for every key whose freq is in the span.
        for (int s = 0; s < SLOTS; ++s) {
            if (!used_[s]) continue;
            float f = key_[s] / 2.0f;
            if (f >= span_lo_mhz && f <= span_hi_mhz) {
                if (seen_[s] < window) ++seen_[s];
            }
        }
        // Record peak levels (ring of `window`).
        for (int i = 0; i < nhits; ++i) {
            int s = ensure_slot(key_of(hits[i]));
            if (s < 0) continue;
            lvl_[s][lvl_head_[s]] = hits[i].peak_dbm;
            lvl_head_[s] = (lvl_head_[s] + 1) % window;
            if (lvl_count_[s] < window) ++lvl_count_[s];
        }
    }

    bool is_artifact(const SpectrumHit& h) const {
        if (h.bw_mhz() > max_artifact_bw_mhz) return false;
        int s = find_slot(key_of(h));
        if (s < 0) return false;
        if (seen_[s] < min_observations || lvl_count_[s] < min_observations) return false;
        if ((float)lvl_count_[s] / (float)seen_[s] < hit_frac) return false;
        float mn = lvl_[s][0], mx = lvl_[s][0];
        for (int i = 1; i < lvl_count_[s]; ++i) {
            if (lvl_[s][i] < mn) mn = lvl_[s][i];
            if (lvl_[s][i] > mx) mx = lvl_[s][i];
        }
        return (mx - mn) <= level_tol_db;
    }

    // Compact hits in place, dropping artifacts. Returns new count.
    int keep(SpectrumHit* hits, int nhits) const {
        int w = 0;
        for (int i = 0; i < nhits; ++i) {
            if (!is_artifact(hits[i])) hits[w++] = hits[i];
        }
        return w;
    }

   private:
    static constexpr int WMAX = 20;
    bool used_[SLOTS] = {};
    int key_[SLOTS] = {};
    int seen_[SLOTS] = {};
    float lvl_[SLOTS][WMAX] = {};
    int lvl_head_[SLOTS] = {};
    int lvl_count_[SLOTS] = {};

    static int key_of(const SpectrumHit& h) {
        // int(round(low+high)) — midpoint on a 0.5 MHz grid (sweep.py:398-402).
        // Banker's rounding to match Python round().
        return sd_round_even(h.low_mhz + h.high_mhz);
    }

    int find_slot(int key) const {
        for (int s = 0; s < SLOTS; ++s)
            if (used_[s] && key_[s] == key) return s;
        return -1;
    }

    int ensure_slot(int key) {
        int s = find_slot(key);
        if (s >= 0) return s;
        for (int i = 0; i < SLOTS; ++i) {
            if (!used_[i]) {
                used_[i] = true; key_[i] = key; seen_[i] = 0;
                lvl_head_[i] = 0; lvl_count_[i] = 0;
                return i;
            }
        }
        return -1;  // table full: stop tracking new artifact keys (bounded, unlike Python)
    }
};

}  // namespace sentinel_detect

#endif  // SENTINEL_DETECT_SD_CFAR_HPP
