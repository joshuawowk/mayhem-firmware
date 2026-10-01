/*
 * ui_sentinel_detect.cpp — Sentinel Detect PortaPack view (GPLv2-or-later).
 *
 * On-device SentinelRF Detector: wideband drone-band sweep + CFAR + lane classification.
 * Sweep engine modelled on ui_looking_glass_app (wideband_spectrum baseband, ChannelSpectrum
 * FIFO drained on DisplayFrameSync). Detection logic is the portable sd_*.hpp cores.
 * RECEIVE-ONLY.
 */
#include "ui_sentinel_detect.hpp"

#include "audio.hpp"

using namespace portapack;

namespace ui::external_app::sentinel_detect {

// MAX_HITS is a member of SentinelDetectView (declared in the header) so the per-frame hit
// buffer can live on the heap-resident view instead of the stack.

// ChannelSpectrum uint8 db -> real dBm, matching Looking Glass's map(v,0,255,-100,20).
static inline float db_from_u8(uint8_t v) {
    return -100.0f + (float)v * (120.0f / 255.0f);
}

SentinelDetectView::SentinelDetectView(NavigationView& nav)
    : nav_{nav} {
    baseband::run_image(portapack::spi_flash::image_tag_wideband_spectrum);
    audio::output::start();

    add_children({&labels,
                  &field_lna,
                  &field_vga,
                  &field_rf_amp,
                  &field_plan,
                  &field_margin,
                  &text_status,
                  &text_threat,
                  &console});
    // Position + register the detection rows (default-constructed; see the header).
    for (int i = 0; i < DETECT_ROWS; ++i) {
        detect_rows_[i].set_parent_rect({0, (4 + i) * 16, 30 * 8, 16});
        add_child(&detect_rows_[i]);
    }

    console.enable_scrolling(true);

    field_plan.set_by_value((int32_t)plan_);
    field_plan.on_change = [this](size_t, OptionsField::value_t v) {
        start_plan((core::ScanPlan)v);
    };

    field_margin.set_value(margin_db_);
    field_margin.on_change = [this](int32_t v) { margin_db_ = v; };

    // Configure the receiver for wideband spectrum, exactly as Looking Glass does.
    receiver_model.set_sampling_rate((uint32_t)SLICE_BW_HZ);
    receiver_model.set_baseband_bandwidth((uint32_t)SLICE_BW_HZ);
    receiver_model.set_squelch_level(0);
    baseband::set_spectrum((size_t)SLICE_BW_HZ, SPECTRUM_TRIGGER);
    receiver_model.enable();

    log_line("Sentinel Detect: RX-only sweep");
    start_plan(plan_);
}

SentinelDetectView::~SentinelDetectView() {
    baseband::spectrum_streaming_stop();
    receiver_model.disable();
    audio::output::stop();
    baseband::shutdown();
}

void SentinelDetectView::focus() {
    field_plan.focus();
}

void SentinelDetectView::log_line(const std::string& s) {
    console.writeln(s);
}

void SentinelDetectView::beep_alert() {
    // Rate-limit to ~1 beep / 0.5 s (30 frame-sync ticks).
    if (frame_sync_ticks_ - last_beep_tick_ < 30) return;
    last_beep_tick_ = frame_sync_ticks_;
    baseband::request_audio_beep(1800, 24000, 150);
}

void SentinelDetectView::start_plan(core::ScanPlan plan) {
    plan_ = plan;
    // Reset all per-band detection state and tracks for the new plan.
    const core::ScanPlanDef& pd = core::SCAN_PLANS[(int)plan_];
    artifact_.reset();
    for (int i = 0; i < MAX_PLAN_BANDS; ++i) {
        if (i < pd.count && pd.bands[i] && pd.bands[i]->fhss)
            hop_[i].init(pd.bands[i]);
    }
    tracks_.reset();
    band_idx_ = 0;
    begin_band(0);
}

void SentinelDetectView::begin_band(int idx) {
    const core::ScanPlanDef& pd = core::SCAN_PLANS[(int)plan_];
    if (pd.count == 0) return;
    band_idx_ = idx % pd.count;
    band_ = pd.bands[band_idx_];

    f_lo_hz_ = (int64_t)(band_->f_lo_mhz * 1e6f);
    f_hi_hz_ = (int64_t)(band_->f_hi_mhz * 1e6f);
    grid_khz_ = band_->bin_width_khz;

    const double grid_hz = grid_khz_ * 1000.0;
    nbins_ = (int)((double)(f_hi_hz_ - f_lo_hz_) / grid_hz);
    if (nbins_ > core::SD_MAX_BINS) nbins_ = core::SD_MAX_BINS;
    if (nbins_ < 4) nbins_ = 4;

    for (int i = 0; i < nbins_; ++i) grid_db_[i] = -110.0f;

    // Slice plan: cover [f_lo, f_hi] with 20 MHz slices stepped by SLICE_STEP_HZ.
    int64_t width = f_hi_hz_ - f_lo_hz_;
    n_slices_ = (int)((width + SLICE_STEP_HZ - 1) / SLICE_STEP_HZ);
    if (n_slices_ < 1) n_slices_ = 1;
    slice_i_ = 0;
    if (n_slices_ == 1)
        f_center_ = (f_lo_hz_ + f_hi_hz_) / 2;
    else
        f_center_ = f_lo_hz_ + SLICE_STEP_HZ / 2;

    retune_slice();
}

void SentinelDetectView::retune_slice() {
    baseband::spectrum_streaming_stop();
    receiver_model.set_target_frequency(f_center_);
    baseband::spectrum_streaming_start();
}

void SentinelDetectView::accumulate_slice(const ChannelSpectrum& spectrum) {
    double each_bin_hz = (spectrum.sampling_rate > 0)
                             ? ((double)spectrum.sampling_rate / 256.0)
                             : ((double)SLICE_BW_HZ / 256.0);
    const double grid_hz = grid_khz_ * 1000.0;
    for (int k = 6; k < 250; ++k) {
        int d = k - 128;
        if (d >= -2 && d <= 2) continue;  // blank DC bins
        int idx = (k + 128) & 255;
        uint8_t v = spectrum.db[idx];
        double f = (double)f_center_ + (double)d * each_bin_hz;
        if (f < (double)f_lo_hz_ || f > (double)f_hi_hz_) continue;
        int cell = (int)((f - (double)f_lo_hz_) / grid_hz);
        if (cell < 0 || cell >= nbins_) continue;
        float db = db_from_u8(v);
        if (db > grid_db_[cell]) grid_db_[cell] = db;
    }
}

void SentinelDetectView::on_channel_spectrum(const ChannelSpectrum& spectrum) {
    // Process one spectrum per tune (Looking Glass pattern): stop, accumulate, advance.
    baseband::spectrum_streaming_stop();
    if (paused_ || band_ == nullptr) return;

    accumulate_slice(spectrum);
    ++slice_i_;

    if (slice_i_ >= n_slices_) {
        finalize_band();
        const core::ScanPlanDef& pd = core::SCAN_PLANS[(int)plan_];
        begin_band((band_idx_ + 1) % (pd.count ? pd.count : 1));
    } else {
        f_center_ += SLICE_STEP_HZ;
        retune_slice();
    }
}

void SentinelDetectView::run_pipeline(const core::SpectrumHit* hits, int nhits) {
    const float t = now_s();
    if (band_->fhss) {
        core::HopState st = hop_[band_idx_].update(hits, nhits, t);
        if (st.fired) {
            core::Detection d = core::make_control(st, *band_);
            bool birth = tracks_.upsert(d, t);
            hop_[band_idx_].reset_debounce();
            if (birth) {
                log_line(std::string(core::lane_badge(d.lane)) + " " +
                         to_string_dec_int(d.band_mhz) + " FHSS h" +
                         to_string_dec_int(d.hops));
            }
            if (d.confidence >= core::ALERT_CONF) beep_alert();
        }
        // Wide carriers sharing an FHSS band are generic energy, not control.
        for (int i = 0; i < nhits; ++i) {
            if (!core::is_control_candidate(hits[i], *band_)) {
                core::Detection d = core::make_sdr_sweep(hits[i], core::Note::wide_in_control_band);
                tracks_.upsert(d, t);
            }
        }
    } else if (band_->lane == core::Lane::video58 || band_->lane == core::Lane::video12) {
        for (int i = 0; i < nhits; ++i) {
            core::Detection d;
            if (core::is_video_candidate(hits[i], *band_)) {
                d = core::make_video(hits[i], *band_, /*confirmed=*/false);  // v1 energy-only
                bool birth = tracks_.upsert(d, t);
                if (birth) {
                    log_line(std::string(core::lane_badge(d.lane)) + " " +
                             to_string_dec_int(d.band_mhz) +
                             (d.channel[0] ? (std::string(" ") + d.channel) : "") + " UNCONF");
                }
                if (d.confidence >= core::ALERT_CONF) beep_alert();
            } else {
                d = core::make_sdr_sweep(hits[i], core::Note::none);
                tracks_.upsert(d, t);
            }
        }
    } else {  // sdr_sweep-lane band (wide_24 / wide_58)
        for (int i = 0; i < nhits; ++i) {
            core::Detection d = core::make_sdr_sweep(hits[i], core::Note::none);
            tracks_.upsert(d, t);
        }
    }
}

void SentinelDetectView::finalize_band() {
    core::SpectrumHit* hits = hits_;  // member scratch (heap), not a stack local
    cfar_.margin_db = (float)margin_db_;
    int n = cfar_.feed(grid_db_, nbins_, band_->f_lo_mhz, grid_khz_ / 1000.0f, hits, MAX_HITS);

    artifact_.observe(hits, n, band_->f_lo_mhz, band_->f_hi_mhz);
    n = artifact_.keep(hits, n);

    run_pipeline(hits, n);

    // Expire stale tracks and log deaths. (died aliases member scratch, not a stack local.)
    core::Track* died = died_;
    int nd = tracks_.expire(now_s(), died, core::TrackTable::MAX_TRACKS);
    for (int i = 0; i < nd; ++i) {
        log_line(std::string("- lost ") + core::lane_badge(died[i].lane) + " " +
                 to_string_dec_int(died[i].band_mhz));
    }

    update_ui();
    draw_strip();
}

void SentinelDetectView::update_ui() {
    // Status: current band + slice progress.
    text_status.set(std::string("BAND ") + band_->name + " " +
                    to_string_dec_int((int)band_->f_lo_mhz) + "-" +
                    to_string_dec_int((int)band_->f_hi_mhz) + " MHz");

    int tc = tracks_.count();
    int threat = (int)(tracks_.aggregate_confidence() * 100.0f + 0.5f);
    std::string th = std::string("THREAT ") + to_string_dec_int(threat) + "%  tracks " +
                     to_string_dec_int(tc);
    if (tracks_.alerting()) th += "  ALERT";
    text_threat.set(th);

    // Detection rows: highest-confidence tracks first (simple selection).
    int shown = 0;
    bool used[core::TrackTable::MAX_TRACKS] = {};
    for (int row = 0; row < DETECT_ROWS; ++row) {
        int best = -1;
        float best_c = -1.0f;
        for (int i = 0; i < tracks_.capacity(); ++i) {
            const core::Track& t = tracks_.at(i);
            if (!t.used || used[i]) continue;
            if (t.confidence > best_c) { best_c = t.confidence; best = i; }
        }
        if (best < 0) { detect_rows_[row].set(""); continue; }
        used[best] = true;
        const core::Track& t = tracks_.at(best);
        std::string s = std::string(core::lane_badge(t.lane)) + " " +
                        to_string_dec_int(t.band_mhz);
        if (t.channel[0]) s += std::string(" ") + t.channel;
        if (t.hopping) s += " h" + to_string_dec_int(t.hops);
        s += " " + to_string_dec_int((int)t.last_rssi_dbm) + "dB " +
             to_string_dec_int((int)(t.confidence * 100.0f + 0.5f)) + "%";
        if (t.video_unconfirmed) s += " U";
        detect_rows_[row].set(s);
        ++shown;
    }
    (void)shown;
}

void SentinelDetectView::draw_strip() {
    // Direct-draw the current band's power spectrum (GlassView-style), 240px wide.
    const int x0 = 0, w = 240;
    const int y0 = strip_y, h = strip_h;
    portapack::display.fill_rectangle({{x0, y0}, {w, h}}, Color::black());
    if (nbins_ <= 0) return;
    const float lo = -100.0f, hi = -30.0f;  // dB display window
    for (int x = 0; x < w; ++x) {
        int cell = x * nbins_ / w;
        if (cell >= nbins_) cell = nbins_ - 1;
        float db = grid_db_[cell];
        float frac = (db - lo) / (hi - lo);
        if (frac < 0.0f) frac = 0.0f;
        if (frac > 1.0f) frac = 1.0f;
        int bar = (int)(frac * h + 0.5f);
        if (bar > 0) {
            portapack::display.fill_rectangle(
                {{x0 + x, y0 + (h - bar)}, {1, bar}}, Color::green());
        }
    }
}

}  // namespace ui::external_app::sentinel_detect
