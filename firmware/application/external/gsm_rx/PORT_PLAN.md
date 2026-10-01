# GSM IMSI-catcher — PortaPack H4M / HackRF One port plan

Port of **[IMSI-catcher](https://github.com/Oros42/IMSI-catcher)** (Oros42) — a GSM IMSI
sniffer — into mayhem-firmware as a standalone on-device **RX external app** (`gsm_rx`,
menu name **"IMSI"**). Tunes a GSM downlink carrier on the HackRF, demodulates it on the
PortaPack, decodes the BCCH/CCCH, and shows live **cell info** (MCC/MNC/LAC/CellID) and
captured **IMSI/TMSI** — no host PC, no gr-gsm, no laptop.

Sibling of `sentinel_labkit` (TX) / `sentinel_detect` (RX sweep). This is a full digital
**receive+decode** app, closest in shape to the in-tree `tetra_rx`.

> Educational / authorized-security-research tool, same framing as the upstream project:
> "made to understand how GSM networks work." GSM (2G) A5/x and the plaintext BCCH/CCCH it
> relies on are legacy; this decodes only the **unencrypted downlink common channels**.

---

## 1. Source of truth (what we are reproducing)

The desktop IMSI-catcher is **three** cooperating pieces; only the last is in this repo:

| Desktop tool | Role | On-device equivalent |
|---|---|---|
| `grgsm_scanner` | sweep the GSM band, find BCCH carriers, read SI3 → cell list | **Scan mode** (v2): spectrum pre-sweep + brief BCCH decode per ARFCN |
| `grgsm_livemon_headless -f <freq>` | park on one C0, GMSK-demod + channel-decode TS0, emit GSMTAP/UDP | **Catch mode**: the `PGSM` baseband image + on-device channel decoder |
| `simple_IMSI-catcher.py` (+ `immediate_assignment_catcher.py`) | parse GSMTAP → IMSI/TMSI/cell | **`gsm_l3.hpp`** portable core (verbatim byte-offset port) |

**The Python in this repo is only the L3 parser.** gr-gsm does the entire GSM physical
layer + channel coding on the desktop. Porting to PortaPack therefore means **re-implementing
the gr-gsm receive chain on the M4** — that is the bulk of the new work, not the parser.

### 1.1 The L3 parser we must reproduce exactly (`simple_IMSI-catcher.py`)

Operates on the **GSMTAP UDP payload** = 16-byte `gsmtap_hdr` + the raw L2 frame. All offsets
below are into that payload (`p[..]`), and must be reproduced **byte-for-byte** — the BCD
nibble-swaps and branch conditions are fragile.

- `gsmtap_hdr`: version, hdr_len, **type**, timeslot, **arfcn** (u16 BE), signal_dbm, snr_db,
  frame_number (u32 BE), **sub_type** (channel type: 1 = BCCH), antenna, sub_slot, res.
- **`find_cell()`** — when `sub_type == 0x01` (BCCH) and `p[0x12] == 0x1b` (System Information
  Type 3): decode `MCC` from `p[0x15]`/`p[0x16]`, `MNC` from `p[0x17]` (BCD nibble games),
  `LAC = p[0x18]<<8 | p[0x19]`, `CellId = p[0x13]<<8 | p[0x14]`.
- **`find_imsi()`** — when `sub_type != 1`: parse **Paging Request Type 1** (`p[0x12]==0x21`)
  and **Type 2** (`p[0x12]==0x22`) with the several mobile-identity layouts enumerated in the
  Python (IMSI vs TMSI, one or two identities, the `0x59`/`0x4d`/`0x4e` pseudo-length
  variants). Emits IMSI (8 bytes BCD) and/or TMSI (4 bytes) per identity.
- `decode_imsi()`: BCD, nibble-swapped, first nibble dropped → `MCC(3) MNC(2..3) MSIN`.
- `str_tmsi()`: 4 bytes → `0xXXXXXXXX`.
- MCC/MNC → country/brand/operator via `mcc-mnc/mcc_codes.json` (a big JSON lookup;
  on-device we ship a **compact subset table** or skip names, see §3.4).
- `immediate_assignment_catcher.py`: **Immediate Assignment** (`p[0x3c]==0x3f`) → SDCCH /
  subchannel / timeslot / hopping / ARFCN. (v3 nicety; not needed for IMSI capture.)

### 1.2 The GSM downlink chain gr-gsm does (what `PGSM` must do)

To catch IMSIs you only need **TS0 of C0** (the BCCH carrier) and two logical channels on it:
**BCCH** (SI Type 3 → cell) and **CCCH/PCH** (Paging Request 1/2/3 → IMSI/TMSI). No hopping,
no ciphering (BCCH/CCCH are always plaintext), no uplink, one timeslot.

- **RF**: GSM900 P-GSM downlink `f = 935.0 + 0.2·ARFCN` MHz, ARFCN 1..124 (EGSM/DCS/PCS later).
  200 kHz channel spacing. GMSK, **270.833 ksym/s** (= 1625/6 kHz), BT = 0.3, 1 bit/symbol.
- **TDMA**: 8 timeslots/frame; timeslot = 156.25 symbols ≈ 576.9 µs; TDMA frame = 4.615 ms.
  TS0 of C0 follows the **51-multiframe** (non-combined): frames {0,10,20,30,40}=**FCCH**,
  {1,11,21,31,41}=**SCH**, {2,3,4,5}=**BCCH**, then CCCH blocks, {50}=idle.
- **Acquisition**:
  - **FCCH** — a burst of all-zero bits → after GMSK a pure tone at **+67.7083 kHz**
    (= +1625/24 kHz) above carrier. Detect it → correct residual frequency offset & get
    coarse timeslot-0 timing.
  - **SCH** — extended 64-bit training sequence (midamble) → correlate to lock symbol/frame
    timing; its 25 info bits (rate-1/2 K=5 conv + CRC-10) give **BSIC** (6b) + **reduced
    frame number** → absolute FN. BSIC's BCC = the **TSC** used by TS0 normal bursts on C0.
- **Normal burst** (BCCH/CCCH): 3 tail + 57 data + 1 stealing + **26 training (TSC)** +
  1 stealing + 57 data + 3 tail + 8.25 guard = 148 active bits. Differential GMSK demod
  `bit = sign(Im(conj(s[n-1])·s[n]))`; TSC correlation gives fine timing & burst validity.
  Two 57-bit data halves → 114 soft/hard bits/burst.
- **Channel coding (xCCH = BCCH & CCCH)**: 184-bit L2 (23 bytes) + **40 FIRE parity**
  (`g(D)=(D²³+1)(D¹⁷+D³+1)`) + 4 tail = 228 → **rate-1/2 K=5 convolutional**
  (G0 = 1+D³+D⁴ = 023₈, G1 = 1+D+D³+D⁴ = 033₈) → 456 coded bits → **block-diagonal
  interleave over 4 consecutive bursts** (456 = 4×114). Receive: de-interleave 4 bursts →
  Viterbi 456→228 → check 40 FIRE parity over the 184 → **23-byte L2 frame**.
- **L2/L3**: LAPDm (`[addr][ctrl][len]` then L3, padded with `0x2B`) → wrap as GSMTAP →
  feed §1.1 parser.

**Acceptance targets** (mirrors gr-gsm on a strong local C0):
- Lock FCCH+SCH on a strong GSM900 BCCH within a few seconds → show ARFCN, **BSIC**, FN.
- Decode **SI Type 3** → **MCC/MNC/LAC/CellID** stable across reads.
- Decode **Paging Request 1/2** → **IMSI** (15 digits, correct MCC/MNC) and **TMSI**.
- FIRE parity rejects all corrupt blocks (zero garbage rows).
- Quiet/no-signal → zero cells, zero IMSIs.

---

## 2. Target: how this maps onto mayhem-firmware

Dual-core LPC43xx: **M0** = application/UI, **M4** = one baseband DSP "image" at a time.
Proven precedents we build directly on:

- **`proc_ais.cpp`** — a real **GMSK** packet demod baseband image (decimate → matched
  filter → clock recovery → slice → NRZI → packet_builder → `application_queue.push`).
  Proves GMSK demod + bit slicing sustains real-time in an M4 image within the 32 KB budget.
- **`tetra_rx`** — a burst-oriented digital RX **external app**: the M4 (`proc_tetra.cpp`)
  does decimation + carrier PLL + **Gardner fractional-NCO symbol timing** + sync-word
  `popcount` correlation and ships **raw channel-coded hard bits** to the M0; the M0
  (`TetraChannelDecoder`) does **de-interleave + Viterbi (16-state K=5) + CRC + L3**.
  GSM's xCCH uses the *same* 16-state K=5 trellis → `tetra_viterbi.cpp` is directly adaptable.
  The fractional phase-accumulator NCO means GSM's awkward 270.833 ksym/s needs **no**
  integer-oversampling resampler.
- **`sentinel_detect`** — the coding conventions this app matches: tiny `main.cpp`, all
  firmware glue in `ui_*.cpp/.hpp`, dependency-free header-only **portable cores** (`sd_*.hpp`)
  that also compile in a **host unit test** (`test_sd_core.cpp`), `app_settings` with an
  `rx_` prefix, `Console`/`Text`/gain-field/`OptionsField`/beep UI.

**Architectural rule (from both TETRA reports): the M4 emits only RAW channel-coded hard
bits; every bit of FEC + L3 runs on the M0.** This keeps the M4 flash/DSP budget small and
makes the entire decoder host-testable off-device.

### 2.1 What's REUSE vs NEW

| Stage | REUSE | NEW |
|---|---|---|
| RF tune + gains | `receiver_model` (`set_target_frequency`/`set_sampling_rate`/`set_baseband_bandwidth`), `RxRadioState`, LNA/VGA/AMP fields | ARFCN↔freq field |
| Baseband scaffold | `BasebandProcessor`+`BasebandThread`+`RSSIThread`, `EventDispatcher`, 2048-sample `buffer_c8_t execute()` | `GSMProcessor` (`proc_gsm.cpp`) |
| Decimate 3.072 MHz → channel rate | `FIRC8xR16x24FS4Decim8` + `FIRC16xR16x32Decim8` | ~200 kHz-passband taps in `dsp_fir_taps.hpp` |
| Carrier PLL + Gardner NCO | structure from `proc_tetra.cpp` | GSM `symbol_phase_inc = 270833·2³²/channel_fs` |
| GMSK demod | `fxpt_atan2` | differential slicer `sign(Im(conj(prev)·cur))` |
| FCCH detect | — | tone hit-counter / short Goertzel at +67.7083 kHz |
| SCH sync | `matched_filter.hpp` correlator + `proc_tetra` sliding-`popcount` register | 64-bit SCH extended training seq, TS0 51-mf scheduler |
| Burst hand-off | `shared_memory.application_queue.push(...)`, `MessageHandlerRegistration` | `Message::ID::GsmBurst` + `GsmBurstMessage` in `common/message.hpp` |
| De-interleave | index-math shape of `tetra_interleave.cpp` | GSM xCCH 4-burst block table |
| Viterbi K=5 | `tetra_viterbi.cpp` 16-state engine | GSM `next_output` (023/033), n_info=228 (xCCH) / 39 (SCH) |
| FIRE / CRC | LFSR pattern of `tetra_crc.cpp` | 40-bit FIRE poly + SCH CRC-10 |
| L3 parse | — | verbatim port of `find_cell`/`find_imsi` |
| Tracks / UI / beep | `sd_tracks.hpp`, `Console`/`Text`/`OptionsField`, `baseband::request_audio_beep` | rekey tracks to identity; IMSI/cell rows |

---

## 3. Architecture of the port

```
firmware/application/external/gsm_rx/
  main.cpp          [APP]  application_information_t (RX, icon, name "IMSI", tag PGSM) + entry
  ui_gsm_rx.hpp/.cpp[APP]  GsmRxView: radio bring-up, PGSM launch, burst handler, UI, wiring
  gsm_bands.hpp     [CORE] ARFCN<->freq (P-GSM/EGSM/DCS/PCS), band tables         [host-test]
  gsm_channel.hpp   [CORE] de-interleave + Viterbi K=5 (023/033) + FIRE-40 + SCH  [host-test]
  gsm_l3.hpp        [CORE] find_cell()/find_imsi() port: SI3 + Paging 1/2/3        [host-test]
  gsm_gsmtap.hpp    [CORE] build the 16-byte GSMTAP header so gsm_l3 offsets match [host-test]
  gsm_tracks.hpp    [CORE] identity table (IMSI/TMSI) + cell table, stale/dedup    [host-test]
  gsm_mcc.hpp       [CORE] compact MCC/MNC → country/operator subset (optional)    [host-test]
  test_gsm_core.cpp [HOST] CHECK-macro harness over captured/synthetic frames (NOT in cmake)
  PORT_PLAN.md      this file

firmware/baseband/
  proc_gsm.cpp/.hpp [BB]   GSMProcessor: decimate→derotate→GMSK slice→FCCH/SCH acq→
                           TS0 scheduler→burst extract→application_queue.push(GsmBurstMessage)

firmware/common/
  message.hpp       [+]    Message::ID::GsmBurst + struct GsmBurstMessage
  spi_image.hpp     [+]    image_tag_gsm_rx {'P','G','S','M'}
firmware/baseband/CMakeLists.txt              [+] set(MODE_CPPSRC proc_gsm.cpp); DeclareTargets(PGSM gsm_rx)
firmware/application/external/external.cmake   [+] EXTCPPSRC += gsm_rx/{main,ui_gsm_rx}.cpp ; EXTAPPLIST += gsm_rx
firmware/application/external/external.ld       [+] ram_external_app_gsm_rx org next-free len 32k + SECTIONS block
firmware/tools/external_app_info.py             [+] register gsm_rx addr + bump external_apps_address_end
```

Two namespaces (load-bearing, sentinel_detect convention): UI in `ui::external_app::gsm_rx`;
portable core in top-level `::gsm_rx` (`namespace core = ::gsm_rx;` alias in the UI header).
The token `gsm_rx` must be **byte-identical** across EXTAPPLIST, dir name,
`ram_external_app_gsm_rx`, the `.external_app_gsm_rx` section + `*(*ui*external_app*gsm_rx*)`
wildcard, and the `main.cpp` section attribute; `'PGSM'` must match `DeclareTargets(PGSM …)`
or `m4_init` panics "NoImg".

### 3.1 `GsmBurstMessage` (M4 → M0 hand-off)

```cpp
struct GsmBurstMessage : Message {          // Message::ID::GsmBurst (next free ID)
    GsmBurstMessage() : Message{ID::GsmBurst} {}
    std::array<uint8_t, 15> data;   // 114 hard bits packed (2×57), MSB-first
    uint32_t frame_number;          // absolute FN from SCH scheduler
    uint16_t arfcn;
    uint8_t  bsic;                  // from SCH (TSC = BCC)
    uint8_t  burst_type;            // FCCH / SCH / NORMAL
    uint8_t  errors;                // TSC correlation hamming distance
};
```
The M0 buffers 4 consecutive NORMAL bursts of the same 51-mf block, runs the `gsm_channel`
decode, and on FIRE-OK wraps the 23 bytes in a GSMTAP header (`gsm_gsmtap`) and calls
`gsm_l3`. SCH bursts feed the SCH decoder to maintain the FN/BSIC lock display.

### 3.2 The hard part, stated honestly

The **`PGSM` GMSK + FCCH/SCH acquisition baseband image is the single novel, make-or-break
deliverable.** Get acquisition (FCCH freq-lock then SCH frame-lock) wrong and nothing
downstream runs. It is also the **only part not verifiable off-device** (no live GSM RF in
CI). Mitigations, all from the recon:
- Start **strong-signal differential** GMSK (no MLSE equalizer) — adequate for a local C0.
- Decode **SI Type 3 first** (predictable frames 2–5 every 51-mf) before paging, to validate
  decimate→demod→sync→deinterleave→Viterbi→FIRE→L3 on a benign channel.
- Reuse the proven `proc_tetra` fractional-NCO timing (no integer-OSR resampler).
- Keep every FEC/L3 stage on the M0 where it is **host-tested** against captured vectors, so
  when a real burst arrives the decoder is already proven correct.

### 3.3 Scan mode (v2) — finding the carrier

`grgsm_scanner`'s job. Two baseband images can't run at once, so: **(a)** sweep with the
`PSPE` wideband_spectrum image (reuse sentinel_detect's sweep) to find persistent narrow
200 kHz carriers → candidate ARFCN list; **(b)** `shutdown()` PSPE, `run_prepared_image`
PGSM, park on the strongest, decode SI3 to confirm it's a BCCH and read the cell. v1 ships
**manual ARFCN entry** + BSIC/FN lock indicator; the sweep-assisted list is v2.

### 3.4 Operator names

`mcc-mnc/mcc_codes.json` is large. On-device: v1 shows numeric **MCC/MNC/LAC/CID + IMSI**
(no operator name) — self-sufficient and matches `--alltmsi` minimalism; `gsm_mcc.hpp` ships
a **compact subset** (a few hundred common MCC/MNC) for a friendly label, expandable from SD.

### 3.5 UI (240×320, model on tetra_rx + sentinel_detect)

```
Row0  LNA_ VGA_ AMP_        (gain fields)          [BAND: GSM900]
Row1  ARFCN [___]  f=947.2MHz     LOCK: FCCH/SCH  BSIC 12  FN 1234567
Row2  CELL  MCC 234 MNC 15  LAC 0x3039  CID 0x619d   -63dBm
Row3.. IMSI list (newest first):
        234150000000000  TMSI 0xd9605460   #3
        ...
Bottom Console log (SI3/paging/parity events)  ·  [Scan] [Catch]  · counter
```
Alert (new IMSI) → guarded `baseband::request_audio_beep`, like fpv_detect/sentinel_detect.

---

## 4. Phased scope

- **v1 — pipeline end-to-end on one manual ARFCN.** New `PGSM` image (differential GMSK,
  FCCH+SCH lock, non-combined TS0 51-mf scheduler). Decode **SI3 → cell** first, then
  **Paging 1/2 → IMSI/TMSI**. Manual ARFCN + gains, Console + cell/IMSI rows, beep.
  Host-test the full FEC+L3 chain against captured GSMTAP/burst vectors.
- **v2 — carrier finding + robustness.** PSPE sweep pre-scan → candidate ARFCN list →
  switch to PGSM; combined-BCCH (CCCH-CONF from SI3/SI4) scheduler variant; Paging Type 3;
  identity dedup/aging table polish; SD logging (CSV, the `-t` path).
- **v3 — reach & interop.** DCS1800/PCS1900/EGSM bands (`OptionsField`); optional 16-state
  **MLSE** equalizer (only if differential BER demands it); optional **GSMTAP-over-USB** so
  the on-device demod feeds Wireshark / the original `simple_IMSI-catcher.py`; Immediate
  Assignment catcher.

**Portable & host-testable now** (`g++ -std=c++17`, fixed arrays, no STL in cores):
`gsm_bands`, `gsm_channel` (deinterleave/Viterbi/FIRE), `gsm_l3`, `gsm_gsmtap`, `gsm_tracks`.
**Hardware-coupled** (device-only): `proc_gsm` (DMA/decimation/PLL/Gardner/sync), ReceiverModel
bring-up, the `application_queue` plumbing.

## 5. Build & verify

Host cores (every commit):
```sh
cd firmware/application/external/gsm_rx
g++ -std=c++17 -O2 test_gsm_core.cpp -o /tmp/tgsm && /tmp/tgsm   # all CHECKs pass
```
Firmware (in-tree, system ARM toolchain — no docker, per sentinel_detect):
```sh
cd firmware/../build           # existing Ninja tree
ninja firmware/baseband/baseband_gsm_rx.elf        # -> PGSM baseband
ninja firmware/application/application.elf         # links the app
cd firmware/baseband && arm-none-eabi-objcopy -O binary baseband_gsm_rx.elf PGSM.bin
cd ../application && arm-none-eabi-objcopy -O binary application.elf application.bin \
    --remove-section=.external_app_*
PYTHONPATH=../../../firmware/tools ../../../firmware/tools/export_external_apps.py \
    ../../../firmware/application . arm-none-eabi-objcopy gsm_rx      # -> gsm_rx.ppma
```
Copy `gsm_rx.ppma` to SD `/APPS`. **Watch the 32 KB `.ppma` ceiling** (app section + raw
`PGSM.bin` + header); keep app-side tables lean, push DSP into the image. (Known unrelated
pre-existing breakage: `baseband_weather`/`baseband_subghzd` fail `_sbrk` at full `ninja` —
independent of this app, which touches neither.)

## 6. Constants (authoritative)

```
GSM900 P-GSM DL f=935.0+0.2·ARFCN MHz (ARFCN 1..124) · 200 kHz spacing
GMSK 270833.33 sym/s (=1625/6 kHz) · BT 0.3 · 1 bit/sym
TDMA: 8 TS · TS=156.25 sym=576.9µs · frame=4.615ms · 51-mf(TS0 C0): 0/10/20/30/40 FCCH,
  1/11/21/31/41 SCH, 2-5 BCCH, 50 idle
FCCH tone = +67708.33 Hz (+1625/24 kHz) · SCH extended training = 64 bits
Normal burst 148b: 3 tail +57 +1 steal +26 TSC +1 steal +57 +3 tail (+8.25 guard) → 114 data
xCCH: 184 L2 (23B) + 40 FIRE + 4 tail = 228 → conv r1/2 K=5 (G0=023,G1=033) → 456 → interleave 4 bursts
FIRE g(D)=(D^23+1)(D^17+D^3+1) · SCH: 25 info +10 CRC +4 tail =39 → conv → 78 (2×39)
Baseband_fs 3.072 MHz (int8 IQ, 2048/buf) · channel demod rate TBD (~1.083–2.166 MHz, 4–8 sps)
L3 (per simple_IMSI-catcher.py offsets into GSMTAP payload): SI3 p[0x12]==0x1b;
  Paging1 p[0x12]==0x21; Paging2 p[0x12]==0x22 · IMSI 8B BCD nibble-swap · TMSI 4B
```

## 7. Decisions (locked for v1)
GSM900 P-GSM default (ARFCN field spans 1..1023 for other bands) · real-time on-device decode ·
new `PGSM` baseband image added · external `.ppma` · differential GMSK (MLSE deferred to v3) ·
non-combined C0 · on-device parse (GSMTAP-over-USB deferred to v3) · manual ARFCN (sweep = v2).

## 8. Build status — v1 COMPLETE, build-verified 2026-07-30

Built in-tree with the system ARM toolchain (`arm-none-eabi-g++` 10.3.1) + the existing
`build/` Ninja tree (cmake auto-reconfigured on the new targets). No docker.

- **Portable cores** (`gsm_bands`, `gsm_channel`, `gsm_gsmtap`, `gsm_l3`, `gsm_tracks`,
  `gsm_mcc`) compile clean on host (`g++ -std=c++17 -O2 -Wall -Wextra`, 0 warnings) and
  **all `test_gsm_core.cpp` checks pass** — ARFCN↔freq, FIRE parity encode/check/reject,
  conv+Viterbi (clean + 3-bit-error correction), full xCCH block round-trip anchored to the
  real SI3 frame, **SI3 → France MCC 208 / MNC 20 / LAC 412 / CID 24989** (byte-exact to
  `simple_IMSI-catcher.py`), Paging Type 1/2 IMSI+TMSI, SCH BSIC+FN round-trip, track
  dedup/count/purge incl. no-recount-after-eviction, TS0 multiframe schedule.
- **`PGSM` baseband** (`proc_gsm.cpp`) → `baseband_gsm_rx.elf`: flash **14,024 B / 32,752
  (42.8%)**, ram 11,380 B / 96 KB. `PGSM.bin` = 14,052 B.
- **`gsm_rx` app** links into `application.elf`: `.external_app_gsm_rx` = **5,252 B / 32 KB**.
- **`gsm_rx.ppma` = 19,308 B / 32,768 (58.9%)** — copy to SD `/APPS` to install.

Registration (mirrors sibling apps): `spi_image.hpp` (`image_tag_gsm_rx {'P','G','S','M'}`) ·
`message.hpp` (`Message::ID::GsmBurst` + `GsmBurstMessage`) · `baseband/CMakeLists.txt`
(`DeclareTargets(PGSM gsm_rx)`) · `external/external.cmake` (EXTCPPSRC + EXTAPPLIST) ·
`external/external.ld` (`ram_external_app_gsm_rx` @ 0xAE0F0000, 32k + SECTIONS) ·
`tools/external_app_info.py` (`external_apps_address_end` → 0xAE100000).

**Adversarial review** (5 dimensions × verify): 3 confirmed issues fixed — split MCC/MNC
zero-padding in the UI; xCCH blocks keyed to their absolute frame (`abs_fn_ - pos`) so a stale
partial can't mix across multiframes; distinct-IMSI count backed by a never-evicted seen-set.
One HIGH "SCH_TRAIN is wrong" finding was **rejected** — the constant `0xB962083E6D45761B`
exactly matches osmocom's authoritative `sb_training` (TS 45.002 §5.2.5); the reviewer's
replacement value was non-standard.

**Not verifiable off-device (expected):** live FCCH/SCH lock + on-air GMSK decode. The M4
acquisition thresholds (`SCH_SYNC_THRESH`, `TSC_THRESH`, `FCCH_MAX_TRANS`), the Gardner clamp
window, and the differential-vs-raw bit sense are the tuning surface for a real cell; the M0
FIRE/CRC gate guarantees no garbage frame ever reaches the parser regardless.

## 9. Hardware-in-the-loop test (HackRF One, 2026-07-30)

Harness `gsm_offline_decode.cpp` (`g++ --sim` | `--iq <cap> --rate <Hz>`) runs the REAL demod
(GMSK differential + Gardner timing + SCH-midamble lock + TS0 framing) + the portable cores.

- **Functional PASS (sim):** GMSK-modulate a TS0 stream (FCCH+SCH+BCCH/SI3+CCCH/paging) →
  pipeline locks and decodes **cell 208/20/0x019c/0x619d + IMSI 208150123456789** end-to-end,
  at 8× and ~3× (device-like) oversampling. Confirms differential bit-sense needs no inversion
  and `SCH_TRAIN` locks at 0 errors.
- **HackRF operational:** `hackrf_info` OK (fw v2.4.0). FM-band sweep (positive control) shows
  real stations (97.6/100.0/105.2 MHz, +15 dB) → RX path works.
- **Live GSM sweep:** GSM900 DL (925–960) and DCS1800 DL (1805–1880) show no continuous
  carrier (no bin's min-across-sweeps exceeds the noise floor). Inconclusive for "2G off-air":
  the test antenna was a 2.4 GHz **WiFi antenna**, badly mismatched at 900 MHz. Needs an
  800–1000 MHz antenna (ANT500 telescopic / cellular whip) to conclude.
- **Negative test PASS (real RF):** 2 s live capture @ 942 MHz (RMS 21.7/127) → decoder yields
  **0 SCH-locks, 0 cells, 0 IMSIs** — no false positives from real over-the-air noise (FIRE/CRC
  gate verified on real data).
- **Outstanding:** a real live-BCCH decode — needs a 900 MHz antenna + a 2G cell in range.
  Path is ready: `hackrf_transfer -r cap.iq -f <ARFCN_Hz> -s 3072000 -a 1 -l 40 -g 40 -n <N>`
  then `gsm_offline_decode --iq cap.iq --rate 3072000`.

### Rebuild recipe
```sh
cd build
ninja firmware/baseband/baseband_gsm_rx.elf firmware/application/application.elf
cd firmware
arm-none-eabi-objcopy -O binary baseband/baseband_gsm_rx.elf baseband/PGSM.bin
arm-none-eabi-objcopy -O binary application/application.elf application/application.bin --remove-section=.external_app_*
../../firmware/tools/export_external_apps.py ../../firmware/application . arm-none-eabi-objcopy gsm_rx
# -> build/firmware/application/gsm_rx.ppma
g++ -std=c++17 -O2 -Wall -Wextra -o /tmp/t \
  ../../firmware/application/external/gsm_rx/test_gsm_core.cpp && /tmp/t   # host cores
