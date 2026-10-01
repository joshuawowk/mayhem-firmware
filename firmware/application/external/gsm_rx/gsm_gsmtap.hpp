/*
 * gsm_gsmtap.hpp — build the 16-byte GSMTAP v2 header so gsm_l3's offsets match the desktop
 * IMSI-catcher (simple_IMSI-catcher.py indexes into the GSMTAP UDP payload).
 *
 * Copyright (C) 2026 (IMSI-catcher PortaPack port)  ·  GPLv2-or-later.  Portable, header-only.
 *
 * On-device we do NOT send UDP; we build the exact GSMTAP payload (16-byte header + 23-byte L2
 * frame = 39 bytes) in a buffer and hand it straight to gsm_l3, so the fragile byte offsets
 * (p[0x12] == message type, etc.) are byte-identical to gr-gsm's output.
 */
#ifndef GSM_RX_GSM_GSMTAP_HPP
#define GSM_RX_GSM_GSMTAP_HPP

#include <cstdint>
#include <cstring>

namespace gsm_rx {

static constexpr int GSMTAP_HDR_LEN = 16;
static constexpr int GSMTAP_PAYLOAD_LEN = GSMTAP_HDR_LEN + 23;  // 39: header + one L2 frame

static constexpr uint8_t GSMTAP_VERSION = 2;
static constexpr uint8_t GSMTAP_TYPE_UM = 0x01;

// channel (sub_type) — the desktop parser keys IMSI/cell decoding off sub_type == BCCH(1).
static constexpr uint8_t GSMTAP_CHANNEL_BCCH = 0x01;
static constexpr uint8_t GSMTAP_CHANNEL_CCCH = 0x02;
static constexpr uint8_t GSMTAP_CHANNEL_PCH = 0x05;

// arfcn flag bits (top of the 16-bit arfcn field).
static constexpr uint16_t GSMTAP_ARFCN_F_PCS = 0x8000;
static constexpr uint16_t GSMTAP_ARFCN_F_UPLINK = 0x4000;

// Fill `out` (>= GSMTAP_PAYLOAD_LEN bytes) with header + the 23-byte L2 frame.
inline void build_gsmtap(uint8_t* out,
                         const uint8_t l2[23],
                         uint8_t channel,       // GSMTAP_CHANNEL_*
                         uint16_t arfcn,        // low 14 bits + optional flags
                         uint32_t frame_number,
                         uint8_t timeslot,
                         int8_t signal_dbm,
                         int8_t snr_db) {
    memset(out, 0, GSMTAP_HDR_LEN);
    out[0] = GSMTAP_VERSION;
    out[1] = GSMTAP_HDR_LEN / 4;  // header length in 32-bit words
    out[2] = GSMTAP_TYPE_UM;
    out[3] = timeslot;
    out[4] = (uint8_t)(arfcn >> 8);
    out[5] = (uint8_t)(arfcn & 0xff);
    out[6] = (uint8_t)signal_dbm;
    out[7] = (uint8_t)snr_db;
    out[8] = (uint8_t)(frame_number >> 24);
    out[9] = (uint8_t)(frame_number >> 16);
    out[10] = (uint8_t)(frame_number >> 8);
    out[11] = (uint8_t)(frame_number & 0xff);
    out[12] = channel;   // sub_type
    out[13] = 0;         // antenna_nr
    out[14] = 0;         // sub_slot
    out[15] = 0;         // res
    memcpy(out + GSMTAP_HDR_LEN, l2, 23);
}

}  // namespace gsm_rx

#endif  // GSM_RX_GSM_GSMTAP_HPP
