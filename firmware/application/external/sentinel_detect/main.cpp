/*
 * main.cpp — Sentinel Detect PortaPack external app: registration + entry point.
 *
 * GPLv2-or-later. Derivative work of the PortaPack-Mayhem firmware (GPLv2+) and an on-device
 * port of the SentinelRF Detector (drone-sentinel). RX-only passive drone-band scanner; the
 * RX sibling of the sentinel_labkit (TX) app.
 *
 * Structure mirrors the in-tree fpv_detect / sentinel_labkit external apps. Uses the
 * wideband-spectrum baseband image (image_tag_wideband_spectrum = 'PSPE'), the same one the
 * Looking Glass app uses, to sweep and read a power spectrum per band.
 */
#include "ui.hpp"
#include "ui_navigation.hpp"
#include "external_app.hpp"

#include "ui_sentinel_detect.hpp"

namespace ui::external_app::sentinel_detect {
void initialize_app(ui::NavigationView& nav) {
    nav.push<SentinelDetectView>();
}
}  // namespace ui::external_app::sentinel_detect

extern "C" {

__attribute__((section(".external_app.app_sentinel_detect.application_information"), used))
application_information_t _application_information_sentinel_detect = {
    /*.memory_location = */ (uint8_t*)0x00000000,
    /*.externalAppEntry = */ ui::external_app::sentinel_detect::initialize_app,
    /*.header_version = */ CURRENT_HEADER_VERSION,
    /*.app_version = */ VERSION_MD5,

    /*.app_name = */ "Sentinel Detect",
    /*.bitmap_data = */ {
        // 16x16 1bpp icon — a radar sweep (concentric arcs + sweep line).
        0x00, 0x00,
        0xE0, 0x07,
        0x18, 0x18,
        0x04, 0x20,
        0xC2, 0x43,
        0x22, 0x40,
        0x11, 0x88,
        0x91, 0x88,
        0x91, 0x89,
        0x11, 0x88,
        0x22, 0x44,
        0xC2, 0x47,
        0x04, 0x22,
        0x18, 0x19,
        0xE0, 0x0F,
        0x00, 0x00,
    },
    /*.icon_color = */ ui::Color::green().v,
    /*.menu_location = */ app_location_t::RX,
    /*.desired_menu_position = */ -1,

    // Reuses the in-tree wideband spectrum baseband (proc_wideband_spectrum, image tag
    // 'PSPE'), the same image Looking Glass uses. export_external_apps.py bundles
    // firmware/baseband/PSPE.bin for this tag into sentinel_detect.ppma.
    /*.m4_app_tag = portapack::spi_flash::image_tag_wideband_spectrum */ {'P', 'S', 'P', 'E'},
    /*.m4_app_offset = */ 0x00000000,
};
}
