/*
 * ui_gsm_rx.cpp — IMSI-catcher (GSM downlink RX) PortaPack app view.
 * Copyright (C) 2026 (IMSI-catcher PortaPack port)  ·  GPLv2-or-later.
 */
#include "ui_gsm_rx.hpp"
#include "baseband_api.hpp"
#include "string_format.hpp"
#include "portapack.hpp"

using namespace portapack;
using namespace ui;

namespace ui::external_app::gsm_rx {

GsmRxView::GsmRxView(NavigationView& nav)
    : nav_{nav} {
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());

    add_children({&field_arfcn, &field_rf_amp, &field_lna, &field_vga, &rssi, &channel,
                  &text_freq, &text_lock, &text_cell, &text_ids, &text_stats, &console});

    field_arfcn.set_value(arfcn_);
    field_arfcn.on_change = [this](int32_t v) {
        arfcn_ = (uint32_t)v;
        retune();
    };

    // The PGSM baseband image does the actual GMSK demod; the RX front-end is configured as a
    // 3.072 MHz narrowband receive path (same as tetra_rx), tuned to the GSM downlink carrier.
    receiver_model.set_modulation(ReceiverModel::Mode::NarrowbandFMAudio);
    receiver_model.set_sampling_rate(3072000);
    receiver_model.set_baseband_bandwidth(1750000);
    receiver_model.set_squelch_level(0);
    receiver_model.enable();

    retune();
    console.writeln("IMSI catcher: tune a GSM900 BCCH (C0).");
}

void GsmRxView::focus() {
    field_arfcn.focus();
}

void GsmRxView::retune() {
    if (!core::arfcn_valid(band_, arfcn_)) return;
    const int64_t hz = core::arfcn_to_downlink_hz(band_, arfcn_);
    receiver_model.set_target_frequency(hz);

    const uint32_t mhz = (uint32_t)(hz / 1000000);
    const uint32_t frac = (uint32_t)((hz % 1000000) / 100000);
    text_freq.set("ARFCN " + to_string_dec_uint(arfcn_) + " " +
                  to_string_dec_uint(mhz) + "." + to_string_dec_uint(frac) + "MHz");

    // a re-tune invalidates the current lock/framing
    cur_mf_ = -1;
    synced_ = false;
    block_mask_ = 0;
    block_active_ = false;
    text_lock.set("SYNC: --");
}

void GsmRxView::on_burst(const GsmBurstMessage& msg) {
    burst_count_++;

    // each burst message corresponds to one TS0 occurrence == one TDMA frame; advance the
    // multiframe phase (SCH re-anchors it authoritatively below).
    if (cur_mf_ >= 0) {
        cur_mf_ = (cur_mf_ + 1) % 51;
        abs_fn_++;
    }

    if (msg.burst_type == 1) {
        handle_sch(msg.payload.data());
    } else if (msg.burst_type == 0) {
        handle_normal(msg);
    }
    // burst_type 2 (FCCH) and 3 (unknown/filler): frame already advanced, nothing to decode

    text_stats.set("B:" + to_string_dec_uint(burst_count_) +
                   " S:" + to_string_dec_uint(sch_count_) +
                   " D:" + to_string_dec_uint(block_ok_));
}

void GsmRxView::handle_sch(const uint8_t* payload) {
    sch_count_++;
    uint8_t mother[78];
    for (int i = 0; i < 78; i++) mother[i] = core::get_bit(payload, i);

    core::SchInfo si = core::sch_decode(mother);
    if (si.valid) {
        cur_mf_ = (int)(si.fn % 51);
        abs_fn_ = si.fn;
        bsic_ = si.bsic;
        synced_ = true;
        text_lock.set("SYNC BSIC" + to_string_dec_uint(si.bsic) +
                      " FN" + to_string_dec_uint(si.fn));
    }
}

void GsmRxView::handle_normal(const GsmBurstMessage& msg) {
    if (cur_mf_ < 0) return;  // not synced to the multiframe yet

    int start = 0, pos = 0;
    bool bcch = false;
    if (!core::xcch_block_of(cur_mf_, start, pos, bcch)) return;  // FCCH/SCH/idle frame

    // Tie the block to its absolute frame: abs_fn_ - pos is identical for all 4 bursts of one
    // physical block and distinct across multiframes, so a stale partial (from a frame the M4
    // could not classify) can never mix with a later multiframe's bursts at the same in-frame
    // start. FIRE would reject a mixed codeword anyway, but this preserves decode yield.
    const uint32_t this_block_fn = abs_fn_ - (uint32_t)pos;
    if (!block_active_ || this_block_fn != block_fn_) {
        block_fn_ = this_block_fn;
        block_active_ = true;
        block_mask_ = 0;
        block_bcch_ = bcch;
    }
    for (int i = 0; i < 15; i++) block_[pos][i] = msg.payload[i];
    block_mask_ |= (uint8_t)(1 << pos);

    if (block_mask_ == 0x0F) {
        try_decode_block();
        block_mask_ = 0;
        block_active_ = false;
    }
}

void GsmRxView::try_decode_block() {
    uint8_t l2[23];
    if (!core::xcch_decode(block_, l2)) return;  // FIRE parity rejected — never emit garbage
    block_ok_++;

    uint8_t gsmtap[core::GSMTAP_PAYLOAD_LEN];
    const uint8_t chan = block_bcch_ ? core::GSMTAP_CHANNEL_BCCH : core::GSMTAP_CHANNEL_CCCH;
    core::build_gsmtap(gsmtap, l2, chan, (uint16_t)arfcn_, abs_fn_, 0, 0, 0);
    handle_l3(gsmtap, core::GSMTAP_PAYLOAD_LEN);
}

void GsmRxView::handle_l3(const uint8_t* gsmtap, int len) {
    core::GsmParse r = core::gsm_l3_parse(gsmtap, len);

    if (r.has_cell) {
        if (cell_.update(r.cell)) {
            // zero-pad the split fields (MCC=3, MNC=2, +3rd digit for 3-digit-MNC regions) so
            // e.g. Orange France MNC 01 shows "01" not "1", matching the desktop parser.
            std::string mcc = to_string_dec_uint(r.cell.mcc, 3, '0');
            std::string mnc = to_string_dec_uint(r.cell.mnc, 2, '0');
            if (core::mnc_is_3digit(r.cell.mcc) && r.cell.mnc3 <= 9)
                mnc += to_string_dec_uint(r.cell.mnc3);
            std::string cell_line = "MCC" + mcc + " MNC" + mnc +
                                    " LAC" + to_string_hex(r.cell.lac, 4) +
                                    " CID" + to_string_hex(r.cell.cell_id, 4);
            text_cell.set(cell_line);
            const char* ctry = core::mcc_country(r.cell.mcc);
            console.writeln("Cell " + mcc + "/" + mnc +
                            (ctry ? (" " + std::string(ctry)) : ""));
        }
    }

    for (int i = 0; i < r.n_id; i++) {
        const core::GsmIdentity& id = r.id[i];
        const bool is_new = identities_.observe(id, (float)tick_);
        if (id.type == core::GsmIdentity::IMSI) {
            text_ids.set("IMSI: " + to_string_dec_uint(identities_.nb_imsi()));
            if (is_new) {
                console.writeln(std::string(id.imsi));
                baseband::request_audio_beep(1000, 24000, 60);
            }
        } else if (is_new) {
            console.writeln("TMSI 0x" + to_string_hex(id.tmsi, 8));
        }
    }
}

void GsmRxView::on_timer() {
    tick_++;
}

GsmRxView::~GsmRxView() {
    receiver_model.disable();
    baseband::shutdown();
}

}  // namespace ui::external_app::gsm_rx
