/*
 * ui_sentinel_detect.hpp — Sentinel Detect PortaPack view (GPLv2-or-later).
 *
 * On-device port of the SentinelRF Detector (drone-sentinel): a passive wideband drone-band
 * scanner. Sweeps the drone bands on the HackRF via the wideband-spectrum baseband (the
 * Looking Glass path), runs the ported CFAR + lane classifier (sd_*.hpp), and shows live
 * per-lane detections + a scrolling event log. RECEIVE-ONLY: nothing here transmits.
 *
 * Sweep engine modelled on ui_looking_glass_app; UI/state idioms on external/fpv_detect.
 * Detection cores are the portable, host-tested sd_*.hpp headers.
 */
#ifndef SENTINEL_DETECT_UI_HPP
#define SENTINEL_DETECT_UI_HPP

#include <cstdint>

#include "app_settings.hpp"
#include "baseband_api.hpp"
#include "portapack.hpp"
#include "radio_state.hpp"
#include "receiver_model.hpp"
#include "message.hpp"
#include "string_format.hpp"
#include "ui.hpp"
#include "ui_widget.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"  // LNAGainField / VGAGainField / RFAmpField

#include "sd_bands.hpp"
#include "sd_cfar.hpp"
#include "sd_hop.hpp"
#include "sd_lanes.hpp"
#include "sd_tracks.hpp"

namespace ui::external_app::sentinel_detect {

namespace core = ::sentinel_detect;

// Number of on-screen detection rows.
static constexpr int DETECT_ROWS = 6;
// Max bands in any scan plan (see sd_bands.hpp SCAN_PLANS).
static constexpr int MAX_PLAN_BANDS = 4;

class SentinelDetectView : public View {
   public:
    explicit SentinelDetectView(NavigationView& nav);
    ~SentinelDetectView();

    SentinelDetectView(const SentinelDetectView&) = delete;
    SentinelDetectView& operator=(const SentinelDetectView&) = delete;

    void focus() override;
    std::string title() const override { return "Sentinel Detect"; }

   private:
    NavigationView& nav_;
    RxRadioState radio_state_{ReceiverModel::Mode::SpectrumAnalysis};
    // Persists the RX front-end settings (LNA/VGA/amp) for this app. Plan/margin are not
    // bound here to avoid member-init-order clobbering; they keep their code defaults.
    app_settings::SettingsManager settings_{"rx_sentinel_detect", app_settings::Mode::RX};

    // ---- sweep engine ----
    static constexpr int64_t SLICE_BW_HZ = 20000000;   // 20 MHz per FFT slice
    static constexpr int64_t SLICE_STEP_HZ = 18000000; // tile step (±9 MHz usable per slice)
    static constexpr uint8_t SPECTRUM_TRIGGER = 32;    // FFTs accumulated per ChannelSpectrum

    core::ScanPlan plan_ = core::ScanPlan::all;
    int band_idx_ = 0;                 // index into the current plan's bands
    const core::BandScan* band_ = nullptr;

    int64_t f_lo_hz_ = 0, f_hi_hz_ = 0;
    float grid_khz_ = 100.0f;          // current band's bin grid
    int nbins_ = 0;
    int64_t f_center_ = 0;             // current slice centre
    int n_slices_ = 1;                 // slices needed to cover the current band
    int slice_i_ = 0;                  // slices accumulated so far this band pass

    float grid_db_[core::SD_MAX_BINS];

    // ---- detection state ----
    core::CfarDetector cfar_;
    // Single shared artifact filter (not one per band): bands occupy disjoint frequency
    // ranges, so their keys never collide, and one filter fits the 32 KB app RAM region
    // (four filters × ~4.6 KB overflowed it → "out of memory" on launch).
    core::PersistenceFilter artifact_;
    core::HopTracker hop_[MAX_PLAN_BANDS];
    core::TrackTable tracks_;
    int32_t margin_db_ = 12;           // operator-tunable CFAR margin

    // Per-frame detection scratch, held as members (heap) rather than stack locals. The
    // finalize_band → cfar_.feed pipeline runs on the small M4 app-thread stack, and these as
    // locals (hits_ ~0.9 KB + died_ ~1.2 KB, plus CFAR's ~0.8 KB) overflowed it — the app
    // launched then died with an M0 "Guru Meditation: Stack Overflow".
    static constexpr int MAX_HITS = 32;
    core::SpectrumHit hits_[MAX_HITS];
    core::Track died_[core::TrackTable::MAX_TRACKS];

    // ---- time base ----
    uint32_t frame_sync_ticks_ = 0;    // DisplayFrameSync counter (~60 Hz)
    float now_s() const { return (float)frame_sync_ticks_ / 60.0f; }

    ChannelSpectrumFIFO* fifo_ = nullptr;
    bool paused_ = false;
    uint32_t last_beep_tick_ = 0;

    // ---- methods ----
    void start_plan(core::ScanPlan plan);
    void begin_band(int idx);
    void retune_slice();
    void on_channel_spectrum(const ChannelSpectrum& spectrum);
    void accumulate_slice(const ChannelSpectrum& spectrum);
    void finalize_band();
    void run_pipeline(const core::SpectrumHit* hits, int nhits);
    void update_ui();
    void draw_strip();
    void log_line(const std::string& s);
    void beep_alert();

    // ---- widgets ----
    Labels labels{
        {{0 * 8, 0 * 16}, "LNA   VGA   AMP", Theme::getInstance()->fg_light->foreground},
        {{0 * 8, 1 * 16}, "PLAN", Theme::getInstance()->fg_light->foreground},
        {{17 * 8, 1 * 16}, "MRGN", Theme::getInstance()->fg_light->foreground},
    };

    LNAGainField field_lna{{4 * 8, 0 * 16}};
    VGAGainField field_vga{{10 * 8, 0 * 16}};
    RFAmpField field_rf_amp{{16 * 8, 0 * 16}};

    OptionsField field_plan{
        {5 * 8, 1 * 16},
        8,
        {
            {"all     ", (int32_t)core::ScanPlan::all},
            {"all-eu  ", (int32_t)core::ScanPlan::all_eu},
            {"fpv     ", (int32_t)core::ScanPlan::fpv},
            {"control ", (int32_t)core::ScanPlan::control},
            {"wideband", (int32_t)core::ScanPlan::wideband},
        }};

    NumberField field_margin{
        {22 * 8, 1 * 16}, 2, {6, 30}, 1, ' '};

    Text text_status{{0 * 8, 2 * 16, 30 * 8, 16}, "Sweeping..."};
    Text text_threat{{0 * 8, 3 * 16, 30 * 8, 16}, "THREAT --  tracks 0"};

    // Detection rows. Default-constructed (Widgets are non-movable, so an array with
    // brace-initializers won't compile under gcc 9.2.1); positions are set in the ctor.
    Text detect_rows_[DETECT_ROWS];

    // Spectrum strip region (drawn directly, GlassView-style): y 10*16 .. 12*16.
    static constexpr int strip_y = 10 * 16;
    static constexpr int strip_h = 2 * 16;

    Console console{{0, 12 * 16 + 8, 240, 320 - (12 * 16 + 8) - 4}};

    MessageHandlerRegistration message_handler_spectrum_config{
        Message::ID::ChannelSpectrumConfig,
        [this](const Message* const p) {
            const auto message = *reinterpret_cast<const ChannelSpectrumConfigMessage*>(p);
            this->fifo_ = message.fifo;
        }};

    MessageHandlerRegistration message_handler_frame_sync{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            ++this->frame_sync_ticks_;
            if (this->fifo_ && !this->paused_) {
                ChannelSpectrum channel_spectrum;
                while (this->fifo_->out(channel_spectrum)) {
                    this->on_channel_spectrum(channel_spectrum);
                }
            }
        }};
};

}  // namespace ui::external_app::sentinel_detect

#endif  // SENTINEL_DETECT_UI_HPP
