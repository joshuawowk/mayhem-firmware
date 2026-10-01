/*
 * gsm_l3.hpp — GSM L3 message parser: byte-exact port of simple_IMSI-catcher.py.
 *
 * Copyright (C) 2026 (IMSI-catcher PortaPack port)  ·  GPLv2-or-later.  Portable, header-only.
 *
 * Reproduces find_cell() (System Information Type 3 -> MCC/MNC/LAC/CellID) and find_imsi()
 * (Paging Request Type 1/2 -> IMSI/TMSI), plus decode_imsi()/str_tmsi(), operating on the same
 * GSMTAP payload layout the desktop tool parses (built by gsm_gsmtap.hpp). The magic byte
 * offsets (p[0x12] == message type, etc.) and BCD nibble-swaps are copied verbatim from the
 * original because they are fragile — see simple_IMSI-catcher.py for the annotated packet dumps.
 */
#ifndef GSM_RX_GSM_L3_HPP
#define GSM_RX_GSM_L3_HPP

#include <cstdint>
#include <cstring>
#include "gsm_gsmtap.hpp"

namespace gsm_rx {

struct GsmCell {
    bool valid = false;
    uint16_t mcc = 0;      // 3 digits, e.g. 208
    uint16_t mnc = 0;      // 2 digits as the desktop tool decodes it (see note below)
    uint8_t mnc3 = 0xFF;   // optional 3rd MNC digit (0xFF = none); the Python ignores this
    uint16_t lac = 0;
    uint16_t cell_id = 0;
};

struct GsmIdentity {
    enum Type : uint8_t { NONE = 0, IMSI = 1, TMSI = 2 };
    Type type = NONE;
    char imsi[16] = {0};   // up to 15 digits, null-terminated (IMSI only)
    uint16_t mcc = 0;      // parsed from IMSI (IMSI only)
    uint16_t mnc = 0;      // 2-digit split (IMSI only)
    uint32_t tmsi = 0;     // 4 bytes big-endian (TMSI only)
};

struct GsmParse {
    bool has_cell = false;
    GsmCell cell;
    int n_id = 0;
    GsmIdentity id[4];
};

// --- helpers -------------------------------------------------------------------------------

inline uint8_t nib_lo(uint8_t b) { return b & 0x0f; }
inline uint8_t nib_hi(uint8_t b) { return (b >> 4) & 0x0f; }

// decode_imsi(): 8 BCD bytes (nibble-swapped) -> IMSI digit string + mcc + 2-digit mnc.
// nibble order per byte is (low, high); nibble[0] is the odd/even+type indicator (dropped).
inline void decode_imsi(const uint8_t bcd[8], GsmIdentity& out) {
    uint8_t dg[16];
    for (int i = 0; i < 8; ++i) {
        dg[2 * i + 0] = nib_lo(bcd[i]);
        dg[2 * i + 1] = nib_hi(bcd[i]);
    }
    // digits start at dg[1]; stop at a filler nibble (>= 10).
    int n = 0;
    for (int i = 1; i < 16 && n < 15; ++i) {
        if (dg[i] > 9) break;
        out.imsi[n++] = (char)('0' + dg[i]);
    }
    out.imsi[n] = '\0';
    out.mcc = (uint16_t)(dg[1] * 100 + dg[2] * 10 + dg[3]);
    out.mnc = (uint16_t)(dg[4] * 10 + dg[5]);
    out.type = GsmIdentity::IMSI;
}

inline uint32_t decode_tmsi(const uint8_t t[4]) {
    return ((uint32_t)t[0] << 24) | ((uint32_t)t[1] << 16) | ((uint32_t)t[2] << 8) | t[3];
}

// --- find_cell(): System Information Type 3 -> cell identity --------------------------------
inline bool parse_si3(const uint8_t* p, int len, GsmCell& out) {
    if (len < 0x1a) return false;
    if (p[12] != GSMTAP_CHANNEL_BCCH) return false;   // gsm.sub_type == 0x01 (BCCH)
    if (p[0x12] != 0x1b) return false;                // Message Type: SI Type 3

    // MCC: digit1 = lowNib(p[0x15]), digit2 = highNib(p[0x15]), digit3 = lowNib(p[0x16]).
    uint8_t d1 = nib_lo(p[0x15]);
    uint8_t d2 = nib_hi(p[0x15]);
    uint8_t d3 = nib_lo(p[0x16]);
    out.mcc = (uint16_t)(d1 * 100 + d2 * 10 + d3);
    // MNC: digit1 = lowNib(p[0x17]), digit2 = highNib(p[0x17]); optional 3rd = highNib(p[0x16]).
    uint8_t e1 = nib_lo(p[0x17]);
    uint8_t e2 = nib_hi(p[0x17]);
    out.mnc = (uint16_t)(e1 * 10 + e2);
    out.mnc3 = nib_hi(p[0x16]);  // 0xF when the MNC is 2-digit
    out.lac = (uint16_t)((p[0x18] << 8) | p[0x19]);
    out.cell_id = (uint16_t)((p[0x13] << 8) | p[0x14]);
    out.valid = true;
    return true;
}

// --- find_imsi(): Paging Request Type 1/2 -> identities ------------------------------------
// Faithful port of the branch structure in simple_IMSI-catcher.py find_imsi(). Emits each IMSI
// and TMSI present in the message; TMSI<->IMSI association/dedup is handled by gsm_tracks.
inline int parse_paging(const uint8_t* p, int len, GsmIdentity* out, int cap) {
    if (len < GSMTAP_PAYLOAD_LEN) return 0;  // need bytes up to p[0x26] (Paging1 IMSI-2)
    int n = 0;
    auto add_imsi = [&](const uint8_t* bcd) {
        if (n < cap) { GsmIdentity id; decode_imsi(bcd, id); out[n++] = id; }
    };
    auto add_tmsi = [&](const uint8_t* t) {
        if (n < cap) { GsmIdentity id; id.type = GsmIdentity::TMSI; id.tmsi = decode_tmsi(t); out[n++] = id; }
    };

    if (p[0x12] == 0x21) {  // Paging Request Type 1
        if (p[0x14] == 0x08 && (p[0x15] & 0x1) == 0x1) {
            add_imsi(&p[0x15]);  // Mobile Identity 1 = IMSI
            if (p[0x10] == 0x59 && p[0x1E] == 0x08 && (p[0x1F] & 0x1) == 0x1) {
                add_imsi(&p[0x1F]);        // Mobile Identity 2 = IMSI
            } else if (p[0x10] == 0x4d && p[0x1E] == 0x05 && p[0x1F] == 0xf4) {
                add_tmsi(&p[0x20]);        // Mobile Identity 2 = TMSI
            }
        } else if (p[0x1B] == 0x08 && (p[0x1C] & 0x1) == 0x1) {
            add_tmsi(&p[0x16]);            // Mobile Identity 1 = TMSI
            add_imsi(&p[0x1C]);            // Mobile Identity 2 = IMSI
        } else if (p[0x14] == 0x05 && (p[0x15] & 0x07) == 4) {
            add_tmsi(&p[0x16]);            // Mobile Identity 1 = TMSI
            if (p[0x1B] == 0x05 && (p[0x1C] & 0x07) == 4)
                add_tmsi(&p[0x1D]);        // Mobile Identity 2 = TMSI
        }
    } else if (p[0x12] == 0x22) {  // Paging Request Type 2
        if (p[0x1D] == 0x08 && (p[0x1E] & 0x1) == 0x1) {
            add_tmsi(&p[0x14]);            // Mobile Identity 1 = TMSI
            add_tmsi(&p[0x18]);            // Mobile Identity 2 = TMSI
            add_imsi(&p[0x1E]);            // Mobile Identity 3 = IMSI
        }
    }
    return n;
}

// Top-level: parse a GSMTAP payload (header + L2 frame) into cell and/or identities.
inline GsmParse gsm_l3_parse(const uint8_t* payload, int len) {
    GsmParse r;
    if (len < GSMTAP_HDR_LEN + 4) return r;
    if (payload[12] == GSMTAP_CHANNEL_BCCH) {
        if (parse_si3(payload, len, r.cell)) r.has_cell = true;
    } else {
        r.n_id = parse_paging(payload, len, r.id, 4);
    }
    return r;
}

}  // namespace gsm_rx

#endif  // GSM_RX_GSM_L3_HPP
