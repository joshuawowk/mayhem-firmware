/*
 * proc_gsm.cpp — GSM downlink GMSK demod + FCCH/SCH burst-sync baseband image (PGSM).
 * Copyright (C) 2026 (IMSI-catcher PortaPack port)  ·  GPLv2-or-later.
 * See proc_gsm.hpp for the chain overview.
 */
#include "proc_gsm.hpp"
#include "portapack_shared_memory.hpp"
#include "audio_dma.hpp"
#include "event_m4.hpp"

#include <algorithm>

GSMProcessor::GSMProcessor() {
    // ~±100 kHz low-pass /4 FS/4 decimator (shared with proc_capture's 200 kHz mode) —
    // isolates the 200 kHz GSM channel and de-rotates the fs/4 IF to baseband.
    decim_0.configure(taps_200k_decim_0.taps);
    baseband_thread.start();
    configured = true;
}

void GSMProcessor::execute(const buffer_c8_t& buffer) {
    if (!configured) return;

    // 3.072 MHz c8 → 768 kHz c16 (≈2.84 samples/symbol).
    const auto channel_out = decim_0.execute(buffer, dst_buffer_0);
    feed_channel_stats(channel_out);

    for (size_t i = 0; i < channel_out.count; i++) {
        const complex16_t sample = channel_out.p[i];

        const uint32_t old_phase = symbol_phase;
        symbol_phase += symbol_phase_inc;

        // half-symbol strobe (for Gardner)
        if ((old_phase < 0x80000000) && (symbol_phase >= 0x80000000))
            mid_sample = sample;

        // full-symbol strobe
        if (symbol_phase < old_phase) {
            prev_prompt = prompt_sample;
            prompt_sample = sample;

            // Gardner timing-error detector on the complex samples (per proc_tetra).
            const int32_t err_i = mid_sample.real() * (prompt_sample.real() - prev_prompt.real());
            const int32_t err_q = mid_sample.imag() * (prompt_sample.imag() - prev_prompt.imag());
            const int32_t timing_error = err_i + err_q;

            int64_t inc = static_cast<int64_t>(symbol_phase_inc) + (timing_error >> 16);
            if (inc < SYM_INC_MIN) inc = SYM_INC_MIN;
            if (inc > SYM_INC_MAX) inc = SYM_INC_MAX;
            symbol_phase_inc = static_cast<uint32_t>(inc);

            process_symbol(prompt_sample);
        }
    }
}

void GSMProcessor::process_symbol(const complex16_t& cur) {
    // Differential GMSK demod: GSM's differential precoding makes a phase-difference detector
    // recover the channel bits directly. bit = sign(Im(conj(prev)·cur)). Polarity ambiguity is
    // resolved per-burst by the SCH/TSC correlation (inverted flag); wrong sense is caught by
    // the M0 FIRE/CRC gate, so no garbage frames escape.
    const int32_t d = static_cast<int32_t>(prev_symbol.real()) * cur.imag() -
                      static_cast<int32_t>(prev_symbol.imag()) * cur.real();
    prev_symbol = cur;
    on_new_bit(d > 0 ? 1 : 0);
}

uint8_t GSMProcessor::history_bit(uint64_t absolute_bit) const {
    const size_t p = absolute_bit % 2048;
    return (bit_history[p >> 3] >> (7 - (p & 7))) & 1;
}

void GSMProcessor::on_new_bit(uint8_t bit) {
    // append to the 2048-bit history ring
    const size_t idx = bit_count % 2048;
    if ((idx & 7) == 0) bit_history[idx >> 3] = 0;
    bit_history[idx >> 3] |= static_cast<uint8_t>(bit << (7 - (idx & 7)));
    bit_count++;

    // slide the 64-bit sync register (newest bit in the LSB)
    sync_register = (sync_register << 1) | bit;

    // SCH midamble (re)lock: the 64-bit training lands in the sync register the instant its
    // last bit arrives; that pins the burst start (self-corrects the TS0 timeline every SCH).
    if (bit_count >= SCH_TRAIN_BITS) {
        const uint32_t e_pos = __builtin_popcountll(sync_register ^ SCH_TRAIN);
        const uint32_t e_neg = __builtin_popcountll(sync_register ^ ~SCH_TRAIN);
        if (e_pos <= SCH_SYNC_THRESH || e_neg <= SCH_SYNC_THRESH) {
            // last training bit == burst bit (SCH_TRAIN_POS + SCH_TRAIN_BITS - 1) == 105
            const uint64_t last_train_bit = bit_count - 1;
            const uint64_t bs = last_train_bit - (SCH_TRAIN_POS + SCH_TRAIN_BITS - 1);
            next_burst_start = bs;
            locked = true;
        }
    }

    // extract each TS0 burst once its full 148 bits have arrived
    if (locked && bit_count >= next_burst_start + BURST_BITS) {
        extract_and_push();
        next_burst_start += FRAME_SYMBOLS;  // next TS0 is one TDMA frame (1250 symbols) later
    }
}

void GSMProcessor::extract_and_push() {
    const uint64_t bs = next_burst_start;

    // classify: does the 64-bit SCH training sit at burst bits [42..105]?
    uint64_t tr = 0;
    for (uint32_t i = 0; i < SCH_TRAIN_BITS; i++)
        tr = (tr << 1) | history_bit(bs + SCH_TRAIN_POS + i);
    const uint32_t sch_pos = __builtin_popcountll(tr ^ SCH_TRAIN);
    const uint32_t sch_neg = __builtin_popcountll(tr ^ ~SCH_TRAIN);
    const uint32_t sch_err = std::min(sch_pos, sch_neg);
    const bool sch_inv = sch_neg < sch_pos;

    if (sch_err <= SCH_SYNC_THRESH) {
        // SCH burst → 78 coded bits = burst[3..41] ++ burst[106..144]
        std::array<uint8_t, 15> out{};
        uint32_t o = 0;
        auto emit = [&](uint32_t i) {
            uint8_t b = history_bit(bs + i) ^ (sch_inv ? 1 : 0);
            out[o >> 3] |= static_cast<uint8_t>(b << (7 - (o & 7)));
            o++;
        };
        for (uint32_t i = 3; i <= 41; i++) emit(i);
        for (uint32_t i = 106; i <= 144; i++) emit(i);
        shared_memory.application_queue.push(
            GsmBurstMessage(out.data(), 1 /*SCH*/, local_frame, 0, sch_err, sch_inv));
        local_frame = (local_frame + 1) % 51;
        return;
    }

    // FCCH detection: a frequency burst demods to a near-constant bit run (few transitions).
    uint32_t trans = 0;
    uint8_t pb = history_bit(bs + 3);
    for (uint32_t i = 4; i <= 144; i++) {
        uint8_t cb = history_bit(bs + i);
        trans += (cb ^ pb);
        pb = cb;
    }
    if (trans < FCCH_MAX_TRANS) {
        std::array<uint8_t, 15> out{};
        shared_memory.application_queue.push(
            GsmBurstMessage(out.data(), 2 /*FCCH*/, local_frame, 0, 0, false));
        local_frame = (local_frame + 1) % 51;
        return;
    }

    // Normal burst: correlate the 26-bit TSC at [61..86] over all 8 training sequences.
    uint32_t tsc_win = 0;
    for (uint32_t i = 0; i < TSC_BITS; i++)
        tsc_win = (tsc_win << 1) | history_bit(bs + TSC_POS + i);
    uint32_t best_k = 0, best_err = TSC_BITS + 1;
    bool best_inv = false;
    for (uint32_t k = 0; k < 8; k++) {
        const uint32_t ep = __builtin_popcount(tsc_win ^ TSC[k]);
        const uint32_t en = __builtin_popcount(tsc_win ^ (~TSC[k] & TSC_MASK));
        const uint32_t e = std::min(ep, en);
        if (e < best_err) { best_err = e; best_k = k; best_inv = en < ep; }
    }
    if (best_err > TSC_THRESH) {
        // No recognizable burst this frame — still emit a filler so the M0 keeps the
        // one-message-per-TS0-frame cadence its multiframe counter depends on (dropping the
        // message would desync the frame phase until the next SCH re-anchor).
        std::array<uint8_t, 15> filler{};
        shared_memory.application_queue.push(
            GsmBurstMessage(filler.data(), 3 /*unknown*/, local_frame, 0, static_cast<uint8_t>(best_err), false));
        local_frame = (local_frame + 1) % 51;
        return;
    }

    // 114 data bits = burst[3..59] ++ burst[88..144]
    std::array<uint8_t, 15> out{};
    uint32_t o = 0;
    auto emit = [&](uint32_t i) {
        uint8_t b = history_bit(bs + i) ^ (best_inv ? 1 : 0);
        out[o >> 3] |= static_cast<uint8_t>(b << (7 - (o & 7)));
        o++;
    };
    for (uint32_t i = 3; i <= 59; i++) emit(i);
    for (uint32_t i = 88; i <= 144; i++) emit(i);
    shared_memory.application_queue.push(
        GsmBurstMessage(out.data(), 0 /*Normal*/, local_frame, static_cast<uint8_t>(best_k),
                        static_cast<uint8_t>(best_err), best_inv));
    local_frame = (local_frame + 1) % 51;
}

void GSMProcessor::on_message(const Message* const msg) {
    if (msg->id == Message::ID::AudioBeep)
        audio::dma::beep_start(
            reinterpret_cast<const AudioBeepMessage*>(msg)->freq,
            reinterpret_cast<const AudioBeepMessage*>(msg)->sample_rate,
            reinterpret_cast<const AudioBeepMessage*>(msg)->duration_ms);
}

int main() {
    audio::dma::init_audio_out();
    EventDispatcher event_dispatcher{std::make_unique<GSMProcessor>()};
    event_dispatcher.run();
    return 0;
}
