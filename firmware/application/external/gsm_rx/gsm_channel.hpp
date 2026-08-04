/*
 * gsm_channel.hpp — GSM downlink channel coding for BCCH/CCCH (xCCH) + SCH (portable).
 *
 * Copyright (C) 2026 (IMSI-catcher PortaPack port)
 * GPLv2-or-later.
 *
 * Dependency-free (no firmware headers, no STL, fixed buffers) so it compiles both in the M0
 * application and in the host unit test. This is the forward-error-correction the desktop
 * gr-gsm receiver does after GMSK demod, reproduced faithfully from 3GPP TS 45.003:
 *
 *   xCCH (BCCH, PCH/AGCH/CCCH, SDCCH, SACCH):
 *     184-bit L2 (23 bytes)  --FIRE(224,184)-->  +40 parity  --+4 tail-->  228
 *       --conv r1/2 K=5 (G0=023, G1=033)-->  456  --block interleave-->  4 bursts x 114 bits
 *   SCH:
 *     25-bit info  --CRC10-->  +10  --+4 tail-->  39  --conv r1/2-->  78 bits (2 x 39)
 *
 * The Viterbi engine is the same 16-state K=5 traceback as tetra_rx/tetra_viterbi.cpp, at
 * rate 1/2 with GSM polynomials. All decode/encode paths are mutually consistent and are
 * exercised by test_gsm_core.cpp (encode -> interleave -> deinterleave -> decode round-trips,
 * FIRE rejects corrupted blocks).
 */
#ifndef GSM_RX_GSM_CHANNEL_HPP
#define GSM_RX_GSM_CHANNEL_HPP

#include <cstdint>
#include <cstring>

namespace gsm_rx {

// ------- bit helpers: MSB-first packing (bit 0 == 0x80 of byte 0), matches tetra BitVector --

inline uint8_t get_bit(const uint8_t* p, int i) {
    return (p[i >> 3] >> (7 - (i & 7))) & 1;
}
inline void set_bit(uint8_t* p, int i, uint8_t v) {
    if (v)
        p[i >> 3] |= (uint8_t)(1u << (7 - (i & 7)));
    else
        p[i >> 3] &= (uint8_t)~(1u << (7 - (i & 7)));
}

// ------- convolutional code, rate 1/2, K=5, GSM xCCH polynomials -----------------------------
//   G0 = 1 + D^3 + D^4   (octal 023)
//   G1 = 1 + D + D^3 + D^4 (octal 033)
// state s (before feeding u(k)) holds the last 4 inputs: bit0=u(k-1)..bit3=u(k-4).
//   c0 = u(k) ^ u(k-3) ^ u(k-4)
//   c1 = u(k) ^ u(k-1) ^ u(k-3) ^ u(k-4)
//   s' = ((s<<1)|u(k)) & 0xF

// pattern[s][b] = (c0<<1)|c1 for input bit b from state s.
inline uint8_t conv_output(int s, int b) {
    int b3 = (s >> 2) & 1;  // u(k-3)
    int b4 = (s >> 3) & 1;  // u(k-4)
    int b1 = s & 1;         // u(k-1)
    int c0 = b ^ b3 ^ b4;
    int c1 = b ^ b1 ^ b3 ^ b4;
    return (uint8_t)((c0 << 1) | c1);
}

// Encode n_info input bits (INCLUDING the 4 trailing tail zeros) into 2*n_info coded bits.
inline void conv_encode(const uint8_t* in_bits, int n_info, uint8_t* out_bits) {
    int s = 0;
    for (int k = 0; k < n_info; ++k) {
        int b = get_bit(in_bits, k);
        uint8_t pat = conv_output(s, b);
        set_bit(out_bits, 2 * k + 0, (pat >> 1) & 1);
        set_bit(out_bits, 2 * k + 1, pat & 1);
        s = ((s << 1) | b) & 0xF;
    }
}

// Hard-decision Viterbi. `mother` holds 2*n_info received symbols, each 0, 1, or 0xFF
// (erasure / don't-care). Writes n_info decoded bits. `trace` must hold n_info*16 bytes.
// Returns the surviving path metric (0 == perfect).
inline int conv_viterbi(const uint8_t* mother, int n_info, uint8_t* out_bits, uint8_t* trace) {
    const uint16_t INF = 0x3fff;
    uint16_t cost[16], new_cost[16];
    for (int i = 0; i < 16; ++i) cost[i] = INF;
    cost[0] = 0;

    for (int step = 0; step < n_info; ++step) {
        for (int i = 0; i < 16; ++i) new_cost[i] = INF;
        const uint8_t* rx = &mother[step * 2];
        for (int prev = 0; prev < 16; ++prev) {
            if (cost[prev] >= INF) continue;
            for (int bit = 0; bit < 2; ++bit) {
                uint8_t pat = conv_output(prev, bit);
                int next = ((prev << 1) | bit) & 0xF;
                uint16_t d = 0;
                uint8_t e0 = (pat >> 1) & 1;
                uint8_t e1 = pat & 1;
                if (rx[0] != 0xff && rx[0] != e0) ++d;
                if (rx[1] != 0xff && rx[1] != e1) ++d;
                uint16_t total = cost[prev] + d;
                if (total < new_cost[next]) {
                    new_cost[next] = total;
                    trace[step * 16 + next] = (uint8_t)((prev << 1) | bit);
                }
            }
        }
        memcpy(cost, new_cost, sizeof(cost));
    }

    int best = 0;
    for (int i = 1; i < 16; ++i)
        if (cost[i] < cost[best]) best = i;

    int state = best;
    memset(out_bits, 0, (n_info + 7) >> 3);
    for (int step = n_info - 1; step >= 0; --step) {
        uint8_t packed = trace[step * 16 + state];
        uint8_t prev = (packed >> 1) & 0x0F;
        uint8_t bit = packed & 1;
        if (bit) set_bit(out_bits, step, 1);
        state = prev;
    }
    return cost[best];
}

// ------- FIRE (224,184) shortened cyclic parity -----------------------------------------------
//   g(D) = (D^23+1)(D^17+D^3+1) = D^40 + D^26 + D^23 + D^17 + D^3 + 1
// LFSR polynomial remainder mod g (bit 0 of the stream = highest-degree coefficient).
static constexpr uint64_t FIRE_G_LOW = 0x0004820009ULL;  // g without the implicit D^40 term
static constexpr uint64_t FIRE_MASK40 = 0xFFFFFFFFFFULL;

inline uint64_t fire_remainder(const uint8_t* bits, int nbits) {
    uint64_t rem = 0;
    for (int i = 0; i < nbits; ++i) {
        uint64_t out = (rem >> 39) & 1;
        rem = ((rem << 1) | get_bit(bits, i)) & FIRE_MASK40;
        if (out) rem ^= FIRE_G_LOW;
    }
    return rem;
}

// Compute the 40 parity bits for 184 info bits and append them at info[184..223].
inline void fire_encode(uint8_t* info224 /* 184 info in, 40 parity written */) {
    // remainder of info(D)*D^40 mod g  == process 184 info bits then 40 zeros
    uint64_t rem = 0;
    for (int i = 0; i < 184; ++i) {
        uint64_t out = (rem >> 39) & 1;
        rem = ((rem << 1) | get_bit(info224, i)) & FIRE_MASK40;
        if (out) rem ^= FIRE_G_LOW;
    }
    for (int i = 0; i < 40; ++i) {
        uint64_t out = (rem >> 39) & 1;
        rem = (rem << 1) & FIRE_MASK40;
        if (out) rem ^= FIRE_G_LOW;
    }
    // rem now holds the 40-bit parity (D^39..D^0), MSB first
    for (int i = 0; i < 40; ++i)
        set_bit(info224, 184 + i, (uint8_t)((rem >> (39 - i)) & 1));
}

// True if the 224-bit codeword (184 info + 40 parity) is a valid FIRE codeword.
inline bool fire_check(const uint8_t* codeword224) {
    return fire_remainder(codeword224, 224) == 0;
}

// ------- xCCH block interleaving (TS 45.003 §4.1.4), depth 4, non-overlapping ------------------
//   bit c(k) -> burst B = k mod 4, position j = 2*((49*k) mod 57) + ((k mod 8) div 4)
// bursts[b] holds 114 bits (packed, MSB-first). Deinterleave is the inverse mapping.

inline void xcch_interleave(const uint8_t* coded456, uint8_t bursts[4][15]) {
    for (int b = 0; b < 4; ++b) memset(bursts[b], 0, 15);
    for (int k = 0; k < 456; ++k) {
        int B = k & 3;
        int j = 2 * ((49 * k) % 57) + ((k % 8) / 4);
        set_bit(bursts[B], j, get_bit(coded456, k));
    }
}

inline void xcch_deinterleave(const uint8_t bursts[4][15], uint8_t* coded456) {
    memset(coded456, 0, 57);
    for (int k = 0; k < 456; ++k) {
        int B = k & 3;
        int j = 2 * ((49 * k) % 57) + ((k % 8) / 4);
        set_bit(coded456, k, get_bit(bursts[B], j));
    }
}

// ------- high-level xCCH block encode/decode --------------------------------------------------

// Encode a 23-byte (184-bit) L2 frame into 4 bursts of 114 bits. For tests / on-device replay.
inline void xcch_encode(const uint8_t l2[23], uint8_t bursts[4][15]) {
    uint8_t info[29] = {0};  // 228 bits = 184 info + 40 parity + 4 tail (zeros)
    memcpy(info, l2, 23);    // 184 info bits
    fire_encode(info);       // fills bits 184..223
    // bits 224..227 already zero (tail)
    uint8_t coded[57] = {0}; // 456 bits
    conv_encode(info, 228, coded);
    xcch_interleave(coded, bursts);
}

// Decode 4 bursts (114 bits each) into a 23-byte L2 frame. Returns true iff FIRE parity is OK.
// `trace` scratch must be >= 228*16 bytes; a static local is used if none is provided.
inline bool xcch_decode(const uint8_t bursts[4][15], uint8_t l2_out[23]) {
    uint8_t coded[57];
    xcch_deinterleave(bursts, coded);

    // expand to 456 hard symbols (0/1) for the Viterbi
    uint8_t mother[456];
    for (int i = 0; i < 456; ++i) mother[i] = get_bit(coded, i);

    static uint8_t trace[228 * 16];  // ~3.6 KB; single-threaded decode path
    uint8_t info[29] = {0};
    conv_viterbi(mother, 228, info, trace);

    if (!fire_check(info)) return false;   // rejects garbage / uncorrected errors
    memcpy(l2_out, info, 23);
    return true;
}

// ------- SCH: 25 info bits -> BSIC + frame number (display / sync aid) ------------------------
//   CRC10 g(D) = D^10 + D^8 + D^6 + D^5 + D^4 + D^2 + 1  (TS 45.003 §4.7)

static constexpr uint16_t SCH_CRC_G = 0x0175;  // low 10 bits: taps at 8,6,5,4,2,0 (+ implicit D^10)
static constexpr uint16_t SCH_CRC_MASK = 0x03FF;

inline uint16_t sch_crc_remainder(const uint8_t* bits, int nbits) {
    uint16_t rem = 0;
    for (int i = 0; i < nbits; ++i) {
        uint16_t out = (rem >> 9) & 1;
        rem = (uint16_t)(((rem << 1) | get_bit(bits, i)) & SCH_CRC_MASK);
        if (out) rem ^= SCH_CRC_G;
    }
    return rem;
}

// Encode 25 info bits -> 78 coded bits (info + CRC10 + 4 tail, conv r1/2). For tests.
inline void sch_encode(const uint8_t info25[4] /* 25 bits, MSB-first */, uint8_t coded78[10]) {
    uint8_t u[5] = {0};  // 39 bits: 25 info + 10 crc + 4 tail
    for (int i = 0; i < 25; ++i) set_bit(u, i, get_bit(info25, i));
    // crc over the 25 info bits, then 10 zeros to flush
    uint16_t rem = 0;
    for (int i = 0; i < 25; ++i) {
        uint16_t out = (rem >> 9) & 1;
        rem = (uint16_t)(((rem << 1) | get_bit(info25, i)) & SCH_CRC_MASK);
        if (out) rem ^= SCH_CRC_G;
    }
    for (int i = 0; i < 10; ++i) {
        uint16_t out = (rem >> 9) & 1;
        rem = (uint16_t)((rem << 1) & SCH_CRC_MASK);
        if (out) rem ^= SCH_CRC_G;
    }
    for (int i = 0; i < 10; ++i) set_bit(u, 25 + i, (uint8_t)((rem >> (9 - i)) & 1));
    conv_encode(u, 39, coded78);
}

struct SchInfo {
    bool valid;
    uint8_t bsic;   // NCC(3)|BCC(3)
    uint8_t ncc;
    uint8_t bcc;    // == TSC on C0
    uint32_t fn;    // absolute TDMA frame number
};

// Decode 78 SCH coded bits. mother78[] = 78 hard symbols (0/1/0xff).
inline SchInfo sch_decode(const uint8_t* mother78) {
    SchInfo r{};
    static uint8_t trace[39 * 16];
    uint8_t u[5] = {0};
    conv_viterbi(mother78, 39, u, trace);
    if (sch_crc_remainder(u, 35) != 0) return r;  // check 25 info + 10 crc

    auto bits = [&](int off, int n) -> uint32_t {
        uint32_t v = 0;
        for (int i = 0; i < n; ++i) v = (v << 1) | get_bit(u, off + i);
        return v;
    };
    r.bsic = (uint8_t)bits(0, 6);
    r.ncc = (r.bsic >> 3) & 7;
    r.bcc = r.bsic & 7;
    uint32_t t1 = bits(6, 11);
    uint32_t t2 = bits(17, 5);
    uint32_t t3p = bits(22, 3);
    uint32_t t3 = 10 * t3p + 1;
    r.fn = 51u * ((t3 + 26u - t2) % 26u) + t3 + 51u * 26u * t1;
    r.valid = true;
    return r;
}

// ------- TS0 51-multiframe schedule (non-combined C0 downlink, TS 45.002 §7) -----------------
// Frame index within the 51-multiframe -> logical channel. FCCH: 0/10/20/30/40, SCH:
// 1/11/21/31/41, BCCH: 2..5, then 9 CCCH blocks of 4 frames, idle: 50.
enum GsmChan : uint8_t { CHAN_FCCH, CHAN_SCH, CHAN_BCCH, CHAN_CCCH, CHAN_IDLE };

// The 10 xCCH block start frames (BCCH is the first; the rest are CCCH/PCH-AGCH).
static constexpr int XCCH_BLOCK_START[10] = {2, 6, 12, 16, 22, 26, 32, 36, 42, 46};

inline GsmChan chan_for_frame(int mf) {
    if (mf < 0 || mf > 50) return CHAN_IDLE;
    if (mf == 50) return CHAN_IDLE;
    if (mf % 10 == 0) return CHAN_FCCH;
    if (mf % 10 == 1) return CHAN_SCH;
    if (mf >= 2 && mf <= 5) return CHAN_BCCH;
    return CHAN_CCCH;
}

// If `mf` belongs to an xCCH block, fill start_frame/pos(0..3)/is_bcch and return true.
inline bool xcch_block_of(int mf, int& start_frame, int& pos, bool& is_bcch) {
    for (int i = 0; i < 10; ++i) {
        int s = XCCH_BLOCK_START[i];
        if (mf >= s && mf <= s + 3) {
            start_frame = s;
            pos = mf - s;
            is_bcch = (s == 2);
            return true;
        }
    }
    return false;
}

}  // namespace gsm_rx

#endif  // GSM_RX_GSM_CHANNEL_HPP
