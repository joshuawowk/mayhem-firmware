/*
 * ui_gsm_rx.hpp — IMSI-catcher (GSM downlink RX) PortaPack app view.
 * Copyright (C) 2026 (IMSI-catcher PortaPack port)  ·  GPLv2-or-later.
 *
 * The M0 half: launches the PGSM baseband image, tunes a GSM900 BCCH carrier, receives raw
 * bursts from proc_gsm, and runs the portable cores (gsm_channel FEC + gsm_l3 parse) to show
 * live cell info and captured IMSI/TMSI. All decode logic lives in the dependency-free gsm_*.hpp
 * cores (host-tested by test_gsm_core.cpp); this file is the firmware/radio glue only.
 */
#ifndef __UI_GSM_RX_H__
#define __UI_GSM_RX_H__

#include "ui.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "ui_freq_field.hpp"
#include "app_settings.hpp"
#include "radio_state.hpp"
#include "message.hpp"
#include <array>
#include <string>

#include "gsm_bands.hpp"
#include "gsm_channel.hpp"
#include "gsm_gsmtap.hpp"
#include "gsm_l3.hpp"
#include "gsm_tracks.hpp"
#include "gsm_mcc.hpp"

using namespace ui;

namespace ui::external_app::gsm_rx {

// The portable cores live in the top-level ::gsm_rx namespace; alias to avoid the enclosing
// ui::external_app::gsm_rx shadowing it (load-bearing — see PORT_PLAN.md §3).
namespace core = ::gsm_rx;

class GsmRxView : public View {
   public:
    GsmRxView(NavigationView& nav);
    ~GsmRxView();
    void focus() override;
    std::string title() const override { return "IMSI catcher"; }

   private:
    NavigationView& nav_;
    RxRadioState radio_state_{};
    app_settings::SettingsManager settings_{"rx_gsm_rx", app_settings::Mode::RX};

    core::Band band_{core::Band::GSM900};
    uint32_t arfcn_{62};  // default GSM900 ARFCN (947.4 MHz)

    // ---- burst assembly / framing state (M0 owns the authoritative multiframe phase) -----
    int cur_mf_{-1};        // 0..50 within the 51-multiframe; -1 until first SCH lock
    uint32_t abs_fn_{0};    // absolute TDMA frame number (anchored by SCH, +1 per burst)
    bool synced_{false};
    uint8_t bsic_{0xff};

    uint8_t block_[4][15]{};
    uint32_t block_fn_{0};    // absolute frame of the block's pos0 burst (block identity)
    bool block_active_{false};
    uint8_t block_mask_{0};
    bool block_bcch_{false};

    core::IdentityTable identities_{};
    core::CellState cell_{};

    uint32_t tick_{0};      // DisplayFrameSync ticks — coarse time base for aging
    uint32_t burst_count_{0};
    uint32_t sch_count_{0};
    uint32_t block_ok_{0};

    // ---- UI --------------------------------------------------------------------------------
    NumberField field_arfcn{{UI_POS_X(0), UI_POS_Y(0)}, 3, {1, 1023}, 1, ' '};
    RFAmpField field_rf_amp{{UI_POS_X(13), UI_POS_Y(0)}};
    LNAGainField field_lna{{UI_POS_X(15), UI_POS_Y(0)}};
    VGAGainField field_vga{{UI_POS_X(18), UI_POS_Y(0)}};
    RSSI rssi{{UI_POS_X_RIGHT(9), UI_POS_Y(0), UI_POS_WIDTH(9), 4}};
    Channel channel{{UI_POS_X_RIGHT(9), UI_POS_Y(0) + 5, UI_POS_WIDTH(9), 4}};

    Text text_freq{{UI_POS_X(0), UI_POS_Y(1), UI_POS_WIDTH(16), UI_POS_HEIGHT(1)}, "ARFCN --  ---.-MHz"};
    Text text_lock{{UI_POS_X(16), UI_POS_Y(1), UI_POS_WIDTH(14), UI_POS_HEIGHT(1)}, "SYNC: --"};
    Text text_cell{{UI_POS_X(0), UI_POS_Y(2), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, "CELL: ---"};
    Text text_ids{{UI_POS_X(0), UI_POS_Y(3), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, "IMSI: 0"};
    Text text_stats{{UI_POS_X(0), UI_POS_Y(4), UI_POS_MAXWIDTH, UI_POS_HEIGHT(1)}, "B:0 S:0 D:0"};

    Console console{{UI_POS_X(0), UI_POS_Y(5) + 8, UI_POS_MAXWIDTH, screen_height - (UI_POS_Y(6) + 8)}};

    // ---- logic -----------------------------------------------------------------------------
    void retune();
    void on_burst(const GsmBurstMessage& msg);
    void handle_sch(const uint8_t* payload);
    void handle_normal(const GsmBurstMessage& msg);
    void try_decode_block();
    void handle_l3(const uint8_t* gsmtap, int len);
    void on_timer();

    MessageHandlerRegistration message_handler_burst{
        Message::ID::GsmBurst,
        [this](Message* const p) {
            this->on_burst(*static_cast<const GsmBurstMessage*>(p));
        }};
    MessageHandlerRegistration message_handler_frame_sync{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            this->on_timer();
        }};
};

}  // namespace ui::external_app::gsm_rx

#endif  // __UI_GSM_RX_H__
