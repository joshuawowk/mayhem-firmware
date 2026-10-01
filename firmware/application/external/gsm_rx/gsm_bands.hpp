/*
 * gsm_bands.hpp — GSM band / ARFCN↔frequency tables (portable, header-only).
 *
 * Copyright (C) 2026 (IMSI-catcher PortaPack port)
 * GPLv2-or-later.
 *
 * Dependency-free (no firmware headers, no STL) so it also compiles in the host unit test
 * (test_gsm_core.cpp). Reproduces the ARFCN→downlink-frequency mapping that gr-gsm / the GSM
 * air interface (3GPP TS 45.005) define; v1 targets GSM900 P-GSM, others scaffolded.
 */
#ifndef GSM_RX_GSM_BANDS_HPP
#define GSM_RX_GSM_BANDS_HPP

#include <cstdint>

namespace gsm_rx {

// GMSK / TDMA physical-layer constants (authoritative, TS 45.002 / 45.004).
static constexpr double GSM_SYMBOL_RATE = 1625000.0 / 6.0;       // 270833.33.. sym/s
static constexpr double GSM_FCCH_TONE_HZ = 1625000.0 / 24.0;     // +67708.33 Hz above carrier
static constexpr double GSM_CHANNEL_SPACING_HZ = 200000.0;       // 200 kHz
static constexpr int GSM_TS_PER_FRAME = 8;
static constexpr double GSM_TDMA_FRAME_S = 0.004615;             // 4.615 ms
static constexpr int GSM_MULTIFRAME_51 = 51;                     // TS0 control multiframe

enum class Band : uint8_t {
    GSM850,
    GSM900,   // P-GSM (v1 default)
    EGSM900,
    DCS1800,
    PCS1900,
};

struct BandPlan {
    Band band;
    const char* name;
    uint16_t arfcn_lo;
    uint16_t arfcn_hi;
    int64_t base_dl_hz;    // downlink freq at arfcn == arfcn_ref
    uint16_t arfcn_ref;    // reference ARFCN for base_dl_hz
    bool pcs_flag;         // GSMTAP arfcn PCS band flag
};

// Downlink frequency = base + spacing*(arfcn - ref). Ranges per TS 45.005.
//  P-GSM900 : f = 935.0  MHz + 0.2*(n)        n=1..124
//  E-GSM900 : adds n=975..1023 wrapping below 935 (f = 935 + 0.2*(n-1024))
//  DCS1800  : f = 1805.2 MHz + 0.2*(n-512)    n=512..885
//  PCS1900  : f = 1930.2 MHz + 0.2*(n-512)    n=512..810
//  GSM850   : f = 869.2  MHz + 0.2*(n-128)    n=128..251
inline BandPlan band_plan(Band b) {
    switch (b) {
        case Band::GSM850:  return {Band::GSM850,  "GSM850",  128, 251,  869200000LL, 128, false};
        case Band::EGSM900: return {Band::EGSM900, "EGSM900",   0, 124,  935000000LL,   0, false};
        case Band::DCS1800: return {Band::DCS1800, "DCS1800", 512, 885, 1805200000LL, 512, false};
        case Band::PCS1900: return {Band::PCS1900, "PCS1900", 512, 810, 1930200000LL, 512, true};
        case Band::GSM900:
        default:            return {Band::GSM900,  "GSM900",    1, 124,  935000000LL,   0, false};
    }
}

// EGSM low block (ARFCN 975..1023) is the only non-linear case: it wraps below 935 MHz.
inline bool arfcn_valid(Band b, uint16_t arfcn) {
    if (b == Band::EGSM900 && arfcn >= 975 && arfcn <= 1023) return true;
    const BandPlan p = band_plan(b);
    return arfcn >= p.arfcn_lo && arfcn <= p.arfcn_hi;
}

inline int64_t arfcn_to_downlink_hz(Band b, uint16_t arfcn) {
    if (b == Band::EGSM900 && arfcn >= 975 && arfcn <= 1023)
        return 935000000LL + (int64_t)((int)arfcn - 1024) * 200000LL;
    const BandPlan p = band_plan(b);
    return p.base_dl_hz + ((int64_t)arfcn - (int64_t)p.arfcn_ref) * 200000LL;
}

inline int64_t uplink_offset_hz(Band b) {
    switch (b) {
        case Band::DCS1800: return 95000000LL;
        case Band::PCS1900: return 80000000LL;
        default:            return 45000000LL;  // GSM850/900/EGSM
    }
}

// Nearest ARFCN to a downlink frequency (for a sweep→ARFCN mapping). Returns arfcn_lo..arfcn_hi
// (clamped); does not resolve the EGSM low block (only linear range).
inline uint16_t downlink_hz_to_arfcn(Band b, int64_t hz) {
    const BandPlan p = band_plan(b);
    int64_t n = p.arfcn_ref + (hz - p.base_dl_hz + 100000LL) / 200000LL;  // round to nearest
    if (n < p.arfcn_lo) n = p.arfcn_lo;
    if (n > p.arfcn_hi) n = p.arfcn_hi;
    return (uint16_t)n;
}

}  // namespace gsm_rx

#endif  // GSM_RX_GSM_BANDS_HPP
