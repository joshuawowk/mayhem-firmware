/*
 * main.cpp — gsm_rx external app registration (IMSI-catcher for PortaPack).
 * Copyright (C) 2026 (IMSI-catcher PortaPack port)  ·  GPLv2-or-later.
 */
#include "ui_gsm_rx.hpp"
#include "ui_navigation.hpp"
#include "external_app.hpp"

namespace ui::external_app::gsm_rx {
void initialize_app(ui::NavigationView& nav) {
    nav.push<GsmRxView>();
}
}  // namespace ui::external_app::gsm_rx

extern "C" {

__attribute__((section(".external_app.app_gsm_rx.application_information"), used)) application_information_t _application_information_gsm_rx = {
    /*.memory_location = */ (uint8_t*)0x00000000,
    /*.externalAppEntry = */ ui::external_app::gsm_rx::initialize_app,
    /*.header_version = */ CURRENT_HEADER_VERSION,
    /*.app_version = */ VERSION_MD5,

    /*.app_name = */ "IMSI",
    /*.bitmap_data = */ {
        // 16x16 signal-tower / broadcast icon
        0x00, 0x00,
        0x08, 0x10,
        0x04, 0x20,
        0x42, 0x42,
        0x22, 0x44,
        0x92, 0x49,
        0x4A, 0x52,
        0x4A, 0x52,
        0x92, 0x49,
        0x22, 0x44,
        0x42, 0x42,
        0x04, 0x20,
        0x08, 0x10,
        0x01, 0x80,
        0x03, 0xC0,
        0x03, 0xC0,
    },
    /*.icon_color = */ ui::Color::green().v,
    /*.menu_location = */ app_location_t::RX,
    /*.desired_menu_position = */ -1,

    /*.m4_app_tag = portapack::spi_flash::image_tag_gsm_rx */ {'P', 'G', 'S', 'M'},
    /*.m4_app_offset = */ 0x00000000,  // will be filled at compile time
};
}
