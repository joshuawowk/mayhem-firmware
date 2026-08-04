/*
 * proc_gsm.hpp — GSM downlink GMSK demod + FCCH/SCH burst-sync baseband image (PGSM).
 *
 * Copyright (C) 2026 (IMSI-catcher PortaPack port)  ·  GPLv2-or-later.
 *
 * The M4 half of the on-device GSM IMSI-catcher. Runs on the LPC43xx M4 as a standalone
 * baseband image (tag {'P','G','S','M'}); the gsm_rx application (M0) launches it via
 * run_prepared_image and consumes the raw bursts it pushes.
 *
 * Chain (mirrors proc_tetra's proven scaffold, GMSK-adapted):
 *   3.072 MHz c8 --FIRC8xR16x24FS4Decim4 (/4, FS/4 de-rotate)--> 768 kHz c16 channel
 *     --carrier/timing NCO + Gardner TED--> one recovered symbol per bit-period
 *     --differential GMSK demod + differential decode--> hard bit stream
 *     --> bit-history ring + 64-bit sync register
 *     --SCH midamble (64-bit) correlation--> TS0 lock (self-correcting every SCH)
 *     --every 1250 symbols (one TDMA frame)--> extract the TS0 burst, classify
 *        SCH / FCCH / Normal, push GsmBurstMessage (raw channel-coded bits)
 *
 * All FEC (deinterleave/Viterbi/FIRE) + L3 runs on the M0 (gsm_channel/gsm_l3), so this file
 * ships NO decoder — only demod + framing. Acquisition thresholds and the symbol-timing clamp
 * are the parts that need tuning against a live cell (flagged inline); the FIRE/CRC gate on the
 * M0 guarantees no garbage frames survive regardless of bit-sense/polarity here.
 */
#ifndef __PROC_GSM_H__
#define __PROC_GSM_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "message.hpp"
#include "dsp_decimate.hpp"
#include "dsp_fir_taps.hpp"
#include "rssi_thread.hpp"

#include <cstdint>
#include <array>
#include <memory>

class GSMProcessor : public BasebandProcessor {
   public:
    GSMProcessor();

    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const msg);

   private:
    static constexpr size_t baseband_fs = 3072000;
    static constexpr size_t channel_fs = 768000;                 // 3.072 MHz / 4
    static constexpr double symbol_rate = 1625000.0 / 6.0;       // 270833.33 sym/s

    // ---- DSP front-end -------------------------------------------------------------------
    std::array<complex16_t, 512> dst_0{};
    const buffer_c16_t dst_buffer_0{dst_0.data(), dst_0.size()};
    dsp::decimate::FIRC8xR16x24FS4Decim4 decim_0{};

    // ---- symbol-timing NCO + Gardner (structure per proc_tetra) --------------------------
    uint32_t symbol_phase{0};
    uint32_t symbol_phase_inc{(uint32_t)(symbol_rate * 4294967296.0 / channel_fs)};
    // clamp keeps the NCO within ~1% of nominal (2^32 * 270833/768000 ≈ 1.5148e9)
    static constexpr int64_t SYM_INC_MIN = 1500000000;
    static constexpr int64_t SYM_INC_MAX = 1530000000;

    complex16_t prompt_sample{0, 0};
    complex16_t prev_prompt{0, 0};
    complex16_t mid_sample{0, 0};
    complex16_t prev_symbol{0, 0};  // for differential GMSK demod

    // acquisition thresholds (hamming distances) — tune against a live cell
    static constexpr uint32_t SCH_SYNC_THRESH = 8;   // of 64 SCH training bits
    static constexpr uint32_t TSC_THRESH = 5;        // of 26 normal-burst TSC bits
    static constexpr uint32_t FCCH_MAX_TRANS = 10;   // bit transitions across a FCCH tone burst

    // ---- bit stream / sync ---------------------------------------------------------------
    uint64_t sync_register{0};
    std::array<uint8_t, 256> bit_history{};  // 2048-bit ring (>1250+148 burst reach)
    uint64_t bit_count{0};
    bool configured{false};

    // ---- GSM burst geometry (bits within a 148-bit burst, TS 45.002) ---------------------
    static constexpr uint32_t BURST_BITS = 148;
    static constexpr uint32_t SCH_TRAIN_POS = 42;   // SCH 64-bit extended training start
    static constexpr uint32_t SCH_TRAIN_BITS = 64;
    static constexpr uint32_t TSC_POS = 61;         // normal-burst 26-bit TSC start
    static constexpr uint32_t TSC_BITS = 26;
    static constexpr uint32_t FRAME_SYMBOLS = 1250; // 8 timeslots × 156.25 = TS0 recurrence

    static constexpr uint64_t SCH_TRAIN = 0xB962083E6D45761BULL;  // TS 45.002 §5.2.5
    static constexpr uint64_t SCH_TRAIN_MASK = 0xFFFFFFFFFFFFFFFFULL;
    // 8 normal-burst training sequences (26 bits, MSB=first bit), TS 45.002 §5.2.3
    static constexpr uint32_t TSC_MASK = 0x03FFFFFF;
    static constexpr uint32_t TSC[8] = {
        0b00100101110000100010010111,  // TSC0
        0b00101101110111100010110111,  // TSC1
        0b01000011101110100100001110,  // TSC2
        0b01000111101101000100011110,  // TSC3
        0b00011010111001000001101011,  // TSC4
        0b01001110101100000100111010,  // TSC5
        0b10100111110110001010011111,  // TSC6
        0b11101111000100101110111100,  // TSC7
    };

    // ---- framing state -------------------------------------------------------------------
    bool locked{false};
    uint64_t next_burst_start{0};  // absolute bit index of the next TS0 burst start
    uint32_t local_frame{0};       // running mf-frame hint (M0 owns the authoritative value)

    // ---- methods -------------------------------------------------------------------------
    void process_symbol(const complex16_t& sample);
    void demod_bit(const complex16_t& sample);
    void on_new_bit(uint8_t bit);
    uint8_t history_bit(uint64_t absolute_bit) const;
    void extract_and_push();

    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive, false};
    RSSIThread rssi_thread{};
};

#endif  // __PROC_GSM_H__
