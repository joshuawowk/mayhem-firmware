# Sentinel Detect — PortaPack H4M / HackRF One port plan

Port of the **SentinelRF Detector** (`drone-sentinel`) passive SDR drone detector into
mayhem-firmware as a standalone on-device **RX external app**. Sweeps the drone bands on
the HackRF, classifies carriers into lanes, shows live per-lane detections + a scrolling
event log. No host PC.

Sibling of the existing `sentinel_labkit` (TX, safety-gated siggen). This is the RX side.

---

## 1. Source of truth (what we are reproducing)

The host detector's real engine is `drone-sentinel/coordinator/sentinel/sdr/`:
`plan.py` (band plan), `sweep.py` (CFAR peak-find + artifact filter), `lanes.py` +
`gateway.py` (lane classification/dispatch), `hop.py` (FHSS control-link detection),
`iq.py`+`linesync.py`+`linefit.py` (analog-raster confirmation), `models.py`/`fusion.py`
(track model + fusion). Its own comments say it "mirrors the firmware lanes" — so this port
is what those comments point at.

### 1.1 Band plan (plan.py) — the bands we sweep

| Band | lane | f_lo MHz | f_hi MHz | bin kHz | margin dB | FHSS | max_narrow BW MHz | min_video BW MHz | nominal MHz |
|---|---|---|---|---|---|---|---|---|---|
| video58 | video58 | 5645.0 | 5945.0 | 500 | 12 | no | – | 2.0 | – |
| video12 | video12 | 1060.0 | 1380.0 | 500 | 12 | no | – | 2.0 | – |
| control_24 | control_24 | 2400.0 | 2483.5 | 100 | 12 | yes | 3.0 | – | 2440 |
| control_sub US | control_sub | 902.0 | 928.0 | 100 | 12 | yes | 2.0 | – | 915 |
| control_sub EU | control_sub | 863.0 | 870.0 | 100 | 12 | yes | 2.0 | – | 868 |
| control_433 | control_sub | 433.05 | 434.79 | 50 | 12 | yes | 1.0 | – | 433 |
| wide_24 | sdr_sweep | 2400.0 | 2483.5 | 500 | 12 | no | – | – | – |
| wide_58 | sdr_sweep | 5150.0 | 5900.0 | 1000 | 12 | no | – | – | – |

Default scan plan `"all" = (control_sub_US, video12, control_24, video58)`. FPV 5.8 channel
tables (A/B/E/F/R × 8, byte-identical to fpv_detect's `fpv_frequencies`) + FPV 1.2 channels
(1080..1360 step 40) used only for channel *labels*.

### 1.2 Detection algorithm (sweep.py `CfarDetector`)

Per band, on each sweep frame (a power-vs-frequency array):
- **Floor**: chunked median. `chunk = min(32, max(4, n/4))` bins; per-chunk median; then the
  **25th percentile across ±8 neighbouring chunks** is the floor for that chunk's bins.
- **Hot bin**: `power - floor >= margin_db` (12 dB).
- **Cluster**: contiguous hot bins, bridging gaps `<= max_gap_bins+1` (=2, one missing bin ok),
  `min_bins = 2` to call a carrier.
- **SpectrumHit**: `center_mhz` (peak bin, tuning only), `low_mhz`/`high_mhz` (run edges),
  `peak_dbm`, `floor_dbm`, `snr_db = peak-floor`, `n_bins`; `bw_mhz = max(0, high-low)`.
- CFAR is **memoryless across frames** (floor estimated across frequency, not time) — so the
  first frame is already usable and a persistent carrier cannot hide in the floor.

**Artifact filter** (`PersistenceFilter`): suppress a hit with `bw_mhz <= 0.5` seen in
`>= 0.9` of the last `20` frames (min 8 observations) with level spread `<= 6 dB`. Wide
carriers are never suppressed. Key = midpoint on a 0.5 MHz grid.

### 1.3 Lane classification (gateway.py dispatch + lanes.py)

For each surviving hit, by the band it fell in:
- **FHSS band** (control_*): run the **hop tracker** (below). It emits at most one
  `control_*` message per band per frame. Wide hits (`bw > max_narrow_bw`) in the band →
  `sdr_sweep` (`note="wide carrier in a control band"`). Narrow hits produce no message of
  their own — only the hop tracker speaks.
- **video band** (video58/video12): if `bw_mhz >= 2.0` → video candidate → optional raster
  dwell → **video** (confirmed 0.8) or, without confirmation, `sdr_sweep`
  (`note="wide carrier, analog raster not confirmed"`) when `require_sync`, else video
  energy-only (0.7/0.65). If `bw < 2.0` → `sdr_sweep`.
- **sdr_sweep band** (wide_*): every hit → `sdr_sweep`.

**Hop tracker** (hop.py), one per FHSS band: admit only in-band narrow hits
(`bw <= max_narrow_bw`); bucket peak to 1 MHz channels; age TTL then refresh
(`ttl_frames=3`); `hops = distinct live channels`; `hopping = hops >= 3`; 3-of-6 N-of-M
debounce over a 16-bit shift register of "any narrow hit this frame"; emit only when
`fired && active_now`, then zero the window (min 3 frames between messages). `band_mhz` is
the **fixed nominal** (2440/915/868/433) so an FHSS link fuses to ONE track.

**Confidence** (lanes.py): video sync-confirmed 0.8; video energy-only 0.7 (5.8) / 0.65 (1.2);
control hopping (`hops>=3`) 0.6 else 0.55; sdr_sweep 0.5.

`band_mhz` derivation: video = nearest FPV channel within tol (9 MHz 5.8 / 20 MHz 1.2) of the
span **midpoint** else round(midpoint); control = fixed nominal; sdr_sweep =
`stable_band_mhz` (midpoint on 1 MHz grid if `bw<2`, else 5 MHz grid). Always the midpoint,
never the wandering peak bin.

### 1.4 Raster confirmation (iq.py/linesync.py/linefit.py) — Phase 2

30 ms FM dwell at 20 Msps → decimate to baseband ≥ `MIN_FS_HZ = 2.2*15734.264 ≈ 34.6 kHz` →
search line rate over **15400–15900 Hz** (step `max(bin_hz/4, 0.5)`), Goertzel power
normalized `power/(energy*n*0.5)`; `present = ratio >= sync_ratio(0.08)`;
`standard_confident = present && sep_bins>=3` where `sep_bins = 109.264/bin_hz`
(needs ~27 ms). `LINE_NTSC_HZ=15734.264`, `LINE_PAL_HZ=15625.0`. Confirmed ⇒ video 0.8.

### 1.5 Track/fusion layer (models.py/fusion.py) — device-side dedup

The pipeline emits per-frame with **no debounce** on video/sdr_sweep (10–80 msg/s per
carrier); dedup happens downstream. On-device we replace `FusionEngine` with a small
track table: key `(lane, band_mhz)`; birth on first hit; `last_seen=ts`; **death at
`stale_seconds=30`**; display confidence = `1 - Π(1-base)` over corroborating lanes
`+0.06`/extra lane, cap 0.99; **alert at >= 0.6**. `LANE_BASE_CONFIDENCE`: video58 0.7,
video12 0.65, control_* 0.55, sdr_sweep 0.5.

### 1.6 Acceptance criteria (simulate.py / benchmark.py)

- analog FPV 5.8 @ 5800/8 MHz/−62 dBm → `video58`, band 5800, channel `F4` (raster: NTSC/0.8; energy-only: 0.7).
- analog FPV 1.2 @ 1200/8 MHz → `video12`, band 1200, channel `CH4` (PAL/0.8; energy 0.65).
- ELRS 2.4 (11 ch 2404..2474, 0.8 MHz) → `control_24`, band **2440**, FHSS, hops>=3, 0.6, ≤1 msg/3 frames.
- ELRS 900 (13 ch 903..927, 0.5 MHz) → `control_sub`, band **915**, FHSS, 0.6.
- WiFi AP @ 2437/20 MHz → **never** control_24; `sdr_sweep` `note="wide carrier in a control band"`, 0.5.
- digital FPV 5.8 @ 5745/20 MHz → `sdr_sweep` "not confirmed" (raster mode) / mislabeled video (energy-only, harm visible).
- DC spike (narrow, always-on, constant) → suppressed by artifact filter.
- quiet scene → **zero** detections (12 dB margin gives 0 false alarms over realistic noise).

---

## 2. Target: how PortaPack external RX apps work

- **Registration**: add sources to `EXTCPPSRC` and the app id to `EXTAPPLIST` in
  `application/external/external.cmake`. `main.cpp` exports an `application_information_t`
  (name, 16×16 icon, `icon_color`, `menu_location = app_location_t::RX`, `m4_app_tag`).
  `tools/export_external_apps.py` bundles `firmware/baseband/<TAG>.bin` into `.ppma`.
- **Baseband**: `m4_app_tag = {'P','S','P','E'}` = `image_tag_wideband_spectrum`
  (`proc_wideband_spectrum.cpp`, the baseband Looking Glass uses). Gives us `ChannelSpectrum`.
- **Sweep engine** (from `ui_looking_glass_app`): `RxRadioState{ReceiverModel::Mode::SpectrumAnalysis}`;
  subscribe to `Message::ID::ChannelSpectrumConfig` (grab the `ChannelSpectrumFIFO*`) and drain
  `ChannelSpectrum` frames on `Message::ID::DisplayFrameSync`. 256 bins/slice
  (`SPEC_NB_BINS`), slice width ≤ 20 MHz (`LOOKING_GLASS_SLICE_WIDTH_MAX`), bin ≈ 78 kHz.
  App drives the sweep: `receiver_model.set_target_frequency()` steps across a band; per
  step read the 256-bin `ChannelSpectrum`, blank DC + edges, append to a per-band power
  array; a band completes when we've covered `[f_lo, f_hi]`. Power is `uint8_t` (scaled dB).
- **Fork base for UI/state**: `external/fpv_detect` (Scanning→Candidate→Locked FSM,
  per-channel confidence memory, gain fields, beep, `app_settings`) and `ui_looking_glass_app`
  (sweep + spectrum widget). `ChannelStatistics.max_db` (PWFM) is NOT enough — it gives one
  power per channel with no bandwidth; the PSPE spectrum path is required for lane widths.
- **Conventions** (from `sentinel_labkit`): namespace `ui::external_app::sentinel_detect`,
  GPLv2-or-later header, portable dependency-free core headers (`sl_*.hpp`) that also compile
  in a host unit test, `app_settings::SettingsManager` with an `rx_` prefix.

---

## 3. Architecture of the port

```
sentinel_detect/
  main.cpp                    application_information_t (RX, icon, tag PSPE) + entry
  ui_sentinel_detect.hpp/.cpp  SentinelDetectView: sweep driver + UI + wiring
  sd_bands.hpp                band plan table (§1.1) + FPV channel tables/labels   [portable]
  sd_cfar.hpp                 chunked-median floor + hot-bin cluster → SpectrumHit  [portable]
  sd_artifact.hpp             PersistenceFilter                                     [portable]
  sd_lanes.hpp                width predicates + confidence + band_mhz snapping     [portable]
  sd_hop.hpp                  HopTracker (TTL + 3-of-6 debounce)                    [portable]
  sd_tracks.hpp               track table (30 s stale, prob-OR confidence, alert)   [portable]
  sd_linesync.hpp             line-rate Goertzel search (Phase 2, host-testable)    [portable]
  PORT_PLAN.md                this file
```

The `sd_*.hpp` cores are **pure C++ (no firmware headers)** — direct ports of the Python,
unit-testable on a host, and where the acceptance math lives. `ui_sentinel_detect.cpp` is the
only file that touches firmware APIs (receiver, spectrum FIFO, widgets).

### 3.1 Sweep driver (the on-device replacement for hackrf_sweep)

State machine per the top-level order in gateway.py (FSM D):
```
for each band in plan (round-robin):
  retune across [f_lo,f_hi] in <=20 MHz steps, accumulating a uint8 power[] array
    (convert ChannelSpectrum bins → dB, blank DC/edge bins, map bin→freq)
  when the band is fully covered → one "frame":
    hits = CFAR(power[], band)                 # sd_cfar
    hits = artifact.keep(hits)                 # sd_artifact
    if band.fhss: hop.update(hits) → maybe control detection; wide hits → sdr_sweep
    else:         per hit: wide→video(energy-only v1) else sdr_sweep
    tracks.upsert(detection); tracks.expire(now)   # sd_tracks
  advance to next band
```
"now"/timestamps come from a frame counter × measured frame period (DisplayFrameSync ticks),
since wall-clock isn't needed — TTLs/stale are expressed in frames or ms.

### 3.2 Raster confirmation — scope decision

Full raster confirmation needs FM-demodulated audio (PWFM path), but the sweep needs the PSPE
spectrum path, and an external app bundles ONE baseband image. Therefore:

- **v1 (this port): energy-only video lanes** — faithful to drone-sentinel's documented
  `--energy-only-video` / firmware `VIDEO58_REQUIRE_SYNC=0` mode. Wide carriers in a video
  band are reported `video58`/`video12` at 0.7/0.65 with `detail sync="unconfirmed"`, clearly
  labeled UNCONFIRMED in the UI. Digital-FPV/WiFi-vs-analog is not distinguished (documented
  limitation, exactly as the source documents it). `sd_linesync.hpp` is still ported and
  host-tested so Phase 2 is ready.
- **Phase 2 (follow-on): raster confirm** via a dwell that swaps to the WFM-audio baseband,
  FM-demods the carrier, runs `sd_linesync` on the audio, caches the result 30 s, then swaps
  back to PSPE and resumes the sweep. Baseband-swap feasibility in an external app is the open
  risk; flagged, not attempted in v1.

### 3.3 UI (240×320, model on fpv_detect + Looking Glass)

```
Row0  LNA_ VGA_ AMP_            (gain fields)          [PLAN: all/fpv/ctrl]
Row1  BAND: <name>  <f_lo-f_hi>   SWEEP xx%           status
Row2  spectrum strip (current band power, waterfall-ish)  ~height 5 rows
Row3.. DETECTIONS (per active track):
        [video58 ] 5800 F4  -62dBm  0.70 UNCONF
        [control ] 2440 FHSS h5     -72dBm 0.60
        [sdr     ] 5745 20MHz       -58dBm 0.50
Bottom Console log (scrolling births/deaths), + margin NumberField
```
Widgets: `Labels`, `LNAGainField`/`VGAGainField`/`RFAmpField`, `OptionsField` (plan),
`NumberField` (margin dB), `Text` rows for tracks, `Console` for the log, a spectrum strip
(reuse Looking Glass `add_spectrum_pixel` idea or a simple power bar). Alert (>=0.6) → beep
(`baseband::request_audio_beep`, guarded like fpv_detect) + color.

---

## 4. Build & verify

- Add to `external/external.cmake`: `EXTCPPSRC += external/sentinel_detect/main.cpp,
  ui_sentinel_detect.cpp`; `EXTAPPLIST += sentinel_detect`.
- Build the firmware app the repo's usual way (docker/cmake). The app compiles into the
  external-app set; `export_external_apps.py` produces `sentinel_detect.ppma` bundling
  `PSPE.bin`.
- Host-test the `sd_*.hpp` cores against the simulate.py scenarios (§1.6) with a tiny
  standalone `main` (optional but recommended — the cores are firmware-independent).
- Verify: quiet → 0 tracks; a wide 2.4 GHz carrier → sdr_sweep not control; a hopping narrow
  set → control_24 @ 2440; a wide 5.8 carrier → video58 energy-only.

## 5. Constants (authoritative — from the engine spec)

margin 12 dB · min_bins 2 · max_gap 1 · chunk 32 · coarse span ±8 chunks · coarse pctl 0.25
· artifact window 20 / min_obs 8 / hit_frac 0.9 / level_tol 6 dB / max_bw 0.5 MHz
· hop channel 1 MHz / ttl 3 / hop_min 3 / debounce 3-of-6
· conf: video_sync 0.8 / video58_energy 0.7 / video12_energy 0.65 / control_hop 0.6 /
  control_plain 0.55 / sdr_sweep 0.5
· track stale 30 s · alert 0.6 · lane-base: v58 0.7 v12 0.65 ctrl 0.55 sweep 0.5
· raster: sync_ratio 0.08 · search 15400–15900 Hz · NTSC 15734.264 · PAL 15625 · MIN_FS 34.6 kHz
```

---

## 6. Build status (verified 2026-07-29)

Built in-tree with the system ARM toolchain (`arm-none-eabi-g++` 10.3.1) + the existing
`build/` Ninja tree — no docker needed.

- `main.cpp` and `ui_sentinel_detect.cpp` **compile cleanly** against the real firmware
  headers (only cosmetic `-Weffc++` member-init-list warnings, as with sibling apps).
- Linked into `application.elf`; the app occupies **8,768 B / 32 KB (26.76%)** of its
  external-app RAM region.
- The `wideband_spectrum` baseband builds → `PSPE.bin` (12,652 B).
- `export_external_apps.py` produced **`sentinel_detect.ppma` = 21,424 B** (app + PSPE
  baseband + header), well under the 32 KB `.ppma` limit.
- The portable cores are covered by a host unit test (`test_sd_core.cpp`) that reproduces
  the `simulate.py` acceptance scenes — **all pass**:
  `g++ -std=c++17 -O2 test_sd_core.cpp && ./a.out`.

### Known pre-existing breakage (NOT caused by this port)
A full `ninja` fails to link two **unrelated** baseband images —
`baseband_weather.elf` and `baseband_subghzd.elf` — with `undefined reference to _sbrk`
(they overflow flash / pull in heap; `weather` reports flash 97.48%). This port adds only
an *application* external app + registration and touches **no** baseband, so these are
independent. To build just this app end-to-end without those targets:

```sh
cd build
ninja firmware/baseband/baseband_wideband_spectrum.elf        # PSPE baseband (standard)
ninja firmware/application/application.elf                     # links the app in
cd firmware/baseband && arm-none-eabi-objcopy -O binary baseband_wideband_spectrum.elf PSPE.bin
cd ../application
arm-none-eabi-objcopy -O binary application.elf application.bin --remove-section=.external_app_*
PYTHONPATH=../../../firmware/tools ../../../firmware/tools/export_external_apps.py \
    ../../../firmware/application . arm-none-eabi-objcopy sentinel_detect
# -> sentinel_detect.ppma ; copy to the SD card's /APPS to install.
```

### Registration points (all mirror sibling app `sentinel_labkit`)
`external/external.cmake` (EXTCPPSRC + EXTAPPLIST) · `external/external.ld` (32 KB MEMORY
region + SECTIONS block at 0xAE0E0000) · `tools/external_app_info.py`
(`external_apps_address_end` bumped to 0xAE0F0000).
