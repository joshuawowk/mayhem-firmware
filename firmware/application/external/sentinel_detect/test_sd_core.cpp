/*
 * test_sd_core.cpp — host unit test for the Sentinel Detect portable cores.
 *
 * GPLv2-or-later. Compiles the sd_*.hpp headers on a host (no firmware) and checks them
 * against the SentinelRF acceptance scenarios (drone-sentinel .../sdr/simulate.py):
 *   quiet → 0 hits; analog FPV 5.8 → video58/F4; WiFi in 2.4 → sdr_sweep not control;
 *   ELRS hopping → control_24 @ 2440 fired; DC spike → suppressed; line-sync tone → NTSC.
 *
 * Build:  g++ -std=c++17 -O2 -Wall -o /tmp/test_sd_core test_sd_core.cpp && /tmp/test_sd_core
 */
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "sd_bands.hpp"
#include "sd_cfar.hpp"
#include "sd_hop.hpp"
#include "sd_lanes.hpp"
#include "sd_tracks.hpp"
#include "sd_linesync.hpp"

using namespace sentinel_detect;

static int g_fail = 0;
#define CHECK(cond, msg)                                        \
    do {                                                        \
        if (!(cond)) { std::printf("FAIL: %s\n", msg); ++g_fail; } \
        else { std::printf("ok:   %s\n", msg); }                \
    } while (0)

// Build a per-band power array on the band's native bin grid. Fill with a flat noise floor
// (deterministic pseudo-noise) then paint carriers.
struct Band {
    float f_lo, f_hi, bin_khz;
    int nbins;
    std::vector<float> p;
};

static unsigned s_rng = 12345;
static float frand() {  // 0..1 deterministic
    s_rng = s_rng * 1103515245u + 12345u;
    return ((s_rng >> 16) & 0x7fff) / 32767.0f;
}

static Band make_band(float lo, float hi, float bin_khz, float floor_db, float noise_pp) {
    Band b;
    b.f_lo = lo; b.f_hi = hi; b.bin_khz = bin_khz;
    b.nbins = (int)((hi - lo) / (bin_khz / 1000.0f));
    b.p.assign(b.nbins, 0.0f);
    for (int i = 0; i < b.nbins; ++i) b.p[i] = floor_db + (frand() - 0.5f) * noise_pp;
    return b;
}
static void paint(Band& b, float lo_mhz, float hi_mhz, float level_db) {
    float bin_mhz = b.bin_khz / 1000.0f;
    int i0 = (int)((lo_mhz - b.f_lo) / bin_mhz + 0.5f);
    int i1 = (int)((hi_mhz - b.f_lo) / bin_mhz + 0.5f);
    for (int i = i0; i <= i1 && i < b.nbins; ++i)
        if (i >= 0) b.p[i] = level_db;
}

static int run_cfar(const Band& b, float margin, SpectrumHit* out, int cap) {
    CfarDetector det;
    det.margin_db = margin;
    return det.feed(b.p.data(), b.nbins, b.f_lo, b.bin_khz / 1000.0f, out, cap);
}

int main() {
    SpectrumHit hits[64];

    // --- Scene 1: quiet noise → zero hits (12 dB margin over ~6 dB pp noise) ---------------
    {
        Band b = make_band(5645, 5945, 500, -100.0f, 6.0f);
        int n = run_cfar(b, 12.0f, hits, 64);
        CHECK(n == 0, "quiet 5.8 band yields 0 hits");
    }

    // --- Scene 2: analog FPV 5.8 @ 5800, 8 MHz, -62 dBm → video58 candidate, band 5800/F4 --
    {
        Band b = make_band(5645, 5945, 500, -100.0f, 6.0f);
        paint(b, 5796.0f, 5804.0f, -62.0f);
        int n = run_cfar(b, 12.0f, hits, 64);
        CHECK(n >= 1, "analog FPV 5.8 produces a hit");
        // find widest hit
        int wi = 0; for (int i = 1; i < n; ++i) if (hits[i].bw_mhz() > hits[wi].bw_mhz()) wi = i;
        SpectrumHit& h = hits[wi];
        CHECK(h.bw_mhz() >= 2.0f, "5.8 carrier is wide (>=2 MHz)");
        CHECK(is_video_candidate(h, BAND_VIDEO58), "5.8 wide carrier is a video candidate");
        Detection d = make_video(h, BAND_VIDEO58, /*confirmed=*/false);
        std::printf("      -> lane=%s band=%d ch=%s conf=%.2f bw=%.1f\n",
                    lane_name(d.lane), d.band_mhz, d.channel[0] ? d.channel : "-", d.confidence, d.bw_mhz);
        CHECK(d.lane == Lane::video58, "classified video58");
        CHECK(d.band_mhz == 5800, "video58 band_mhz snapped to 5800");
        CHECK(d.channel[0] == 'F' && d.channel[1] == '4', "video58 channel label F4");
        CHECK(std::fabs(d.confidence - 0.70f) < 1e-4f, "energy-only video58 confidence 0.70");
        CHECK(d.video_unconfirmed, "video58 flagged UNCONFIRMED in energy-only v1");
    }

    // --- Scene 3: WiFi AP @ 2437, 20 MHz in 2.4 control band → sdr_sweep, NOT control -------
    {
        Band b = make_band(2400, 2483.5, 100, -100.0f, 6.0f);
        paint(b, 2427.0f, 2447.0f, -55.0f);
        int n = run_cfar(b, 12.0f, hits, 64);
        CHECK(n >= 1, "WiFi AP produces a hit");
        int wi = 0; for (int i = 1; i < n; ++i) if (hits[i].bw_mhz() > hits[wi].bw_mhz()) wi = i;
        SpectrumHit& h = hits[wi];
        CHECK(h.bw_mhz() > BAND_CONTROL_24.max_narrow_bw_mhz, "WiFi carrier is wide (>3 MHz)");
        CHECK(!is_control_candidate(h, BAND_CONTROL_24), "WiFi carrier is NOT a control candidate");
        Detection d = make_sdr_sweep(h, Note::wide_in_control_band);
        CHECK(d.lane == Lane::sdr_sweep, "WiFi in 2.4 → sdr_sweep");
        CHECK(std::fabs(d.confidence - 0.50f) < 1e-4f, "sdr_sweep confidence 0.50");
    }

    // --- Scene 4: ELRS 2.4 hopping (3 distinct narrow channels over 3 frames) → control ----
    {
        HopTracker hop; hop.init(&BAND_CONTROL_24);
        float chans[3] = {2410.0f, 2430.0f, 2450.0f};
        HopState st{};
        bool fired_any = false;
        for (int f = 0; f < 4; ++f) {
            Band b = make_band(2400, 2483.5, 100, -100.0f, 6.0f);
            // one narrow 0.7 MHz carrier this frame, rotating channel
            float c = chans[f % 3];
            paint(b, c - 0.35f, c + 0.35f, -72.0f);
            int n = run_cfar(b, 12.0f, hits, 64);
            st = hop.update(hits, n, (float)f * 0.1f);  // ~10 sweeps/s
            if (st.fired) { fired_any = true; hop.reset_debounce(); }
        }
        std::printf("      -> hops=%d hopping=%d fired_any=%d\n", st.hops, st.hopping, fired_any);
        CHECK(st.hops >= 3, "hop tracker sees >=3 distinct channels");
        CHECK(st.hopping, "hopping flag set (hops>=hop_min)");
        CHECK(fired_any, "control detection fired within a few frames");
        Detection d = make_control(st, BAND_CONTROL_24);
        CHECK(d.lane == Lane::control_24, "control_24 lane");
        CHECK(d.band_mhz == 2440, "control_24 band_mhz fixed nominal 2440");
        CHECK(std::fabs(d.confidence - 0.60f) < 1e-4f, "hopping control confidence 0.60");
    }

    // --- Scene 5: DC spike (narrow, always-on, constant) → suppressed by artifact filter ---
    {
        PersistenceFilter pf; pf.reset();
        bool suppressed_at_end = false;
        for (int f = 0; f < 12; ++f) {
            Band b = make_band(2400, 2410, 100, -100.0f, 4.0f);
            paint(b, 2404.85f, 2405.15f, -70.0f);  // ~0.3 MHz constant spike
            int n = run_cfar(b, 12.0f, hits, 64);
            pf.observe(hits, n, b.f_lo, b.f_hi);
            int kept = pf.keep(hits, n);
            if (f >= 9) suppressed_at_end = (kept == 0 && n >= 1);
        }
        CHECK(suppressed_at_end, "constant narrow DC spike suppressed after >=8 frames");
    }

    // --- Scene 6: line-sync — a 15734 Hz tone in noise → present/NTSC; pure noise → absent --
    {
        const int N = 4096;
        const float fs = 240000.0f;  // 240 kHz baseband, resolves the line rate
        std::vector<float> tone(N), noise(N);
        for (int i = 0; i < N; ++i) {
            float t = (float)i / fs;
            tone[i] = 0.6f * std::sin(2.0f * (float)M_PI * LINE_NTSC_HZ * t) + (frand() - 0.5f) * 0.2f;
            noise[i] = (frand() - 0.5f) * 1.0f;
        }
        LineRateResult r1 = search_line_rate(tone.data(), N, fs);
        LineRateResult r2 = search_line_rate(noise.data(), N, fs);
        std::printf("      -> tone: present=%d ratio=%.3f std=%s | noise: present=%d ratio=%.3f\n",
                    r1.present, r1.ratio, video_std_name(r1.standard), r2.present, r2.ratio);
        CHECK(r1.present, "15734 Hz raster tone detected (present)");
        CHECK(std::fabs(r1.line_hz - LINE_NTSC_HZ) < 30.0f, "recovered line rate within 30 Hz of NTSC");
        CHECK(!r2.present, "pure noise rejected (not present)");
    }

    // --- Scene 7: fidelity fixes — banker's rounding + n_bins counts hot bins only --------
    {
        // sd_round_even must be round-half-to-even (matches Python round()).
        CHECK(sd_round_even(2400.5f) == 2400, "round_even(2400.5)=2400 (ties to even)");
        CHECK(sd_round_even(2401.5f) == 2402, "round_even(2401.5)=2402 (ties to even)");
        CHECK(sd_round_even(0.5f) == 0 && sd_round_even(1.5f) == 2, "round_even ties to even at 0.5/1.5");
        CHECK(sd_round_even(2.4f) == 2 && sd_round_even(2.6f) == 3, "round_even normal rounding");

        // stable_band_mhz uses banker's rounding on the grid: mid 5152.5 on 5 MHz grid
        // (1030.5 -> even 1030) => 5150, not 5155.
        SpectrumHit wide{}; wide.low_mhz = 5150.0f; wide.high_mhz = 5155.0f; wide.center_mhz = 5152.0f;
        CHECK(stable_band_mhz(wide) == 5150, "stable_band_mhz banker's-rounds 5152.5/5grid -> 5150");

        // n_bins counts hot bins only, not the bridged span. Two hot bins with one
        // sub-threshold bin between them -> n_bins == 2 (span would be 3).
        float p[8];
        for (int i = 0; i < 8; ++i) p[i] = -100.0f;
        p[3] = -60.0f;  // hot
        p[4] = -98.0f;  // gap (not hot, bridged)
        p[5] = -60.0f;  // hot
        SpectrumHit hh[4];
        CfarDetector det; det.margin_db = 12.0f;
        int nh = det.feed(p, 8, 2400.0f, 1.0f, hh, 4);
        CHECK(nh == 1, "bridged hot pair clusters into one hit");
        if (nh == 1) {
            CHECK(hh[0].n_bins == 2, "n_bins counts 2 hot bins (not the 3-wide span)");
            CHECK(hh[0].low_mhz == 2403.0f && hh[0].high_mhz == 2405.0f, "span edges span the bridged gap");
        }
    }

    std::printf("\n%s (%d failure%s)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
