/*
 * test_gsm_core.cpp — host unit test for the GSM IMSI-catcher portable cores.
 *
 * GPLv2-or-later. Compiles the gsm_*.hpp headers on a host (no firmware) and checks them
 * against: (a) the ARFCN↔frequency mapping, (b) channel-coding round-trips (FIRE + Viterbi +
 * xCCH interleave) incl. error correction and garbage rejection, (c) the L3 parser against the
 * annotated packet dumps in simple_IMSI-catcher.py (SI3 -> France 208/20, LAC 412, CID 24989)
 * plus synthetic-but-byte-exact Paging Type 1/2 IMSI/TMSI frames, (d) SCH FN/BSIC round-trip.
 *
 * Build:  g++ -std=c++17 -O2 -Wall -o /tmp/test_gsm_core test_gsm_core.cpp && /tmp/test_gsm_core
 */
#include <cstdio>
#include <cstring>
#include <cstdint>

#include "gsm_bands.hpp"
#include "gsm_channel.hpp"
#include "gsm_gsmtap.hpp"
#include "gsm_l3.hpp"
#include "gsm_tracks.hpp"
#include "gsm_mcc.hpp"

using namespace gsm_rx;

static int g_fail = 0;
#define CHECK(cond, msg)                                             \
    do {                                                             \
        if (!(cond)) { std::printf("FAIL: %s\n", msg); ++g_fail; }   \
        else { std::printf("ok:   %s\n", msg); }                     \
    } while (0)

// Set an MSB-first bit field of `n` bits at bit offset `off` in buffer `p` from value `v`.
static void put_bits(uint8_t* p, int off, int n, uint32_t v) {
    for (int i = 0; i < n; ++i)
        set_bit(p, off + i, (uint8_t)((v >> (n - 1 - i)) & 1));
}

int main() {
    // ---- 1. Bands: ARFCN <-> downlink frequency -------------------------------------------
    CHECK(arfcn_to_downlink_hz(Band::GSM900, 1) == 935200000LL, "GSM900 ARFCN 1 = 935.2 MHz");
    CHECK(arfcn_to_downlink_hz(Band::GSM900, 62) == 947400000LL, "GSM900 ARFCN 62 = 947.4 MHz");
    CHECK(arfcn_to_downlink_hz(Band::DCS1800, 512) == 1805200000LL, "DCS1800 ARFCN 512 = 1805.2 MHz");
    CHECK(arfcn_to_downlink_hz(Band::EGSM900, 975) == 925200000LL, "EGSM900 ARFCN 975 = 925.2 MHz (wraps)");
    CHECK(downlink_hz_to_arfcn(Band::GSM900, 947400000LL) == 62, "947.4 MHz -> ARFCN 62");
    CHECK(!arfcn_valid(Band::GSM900, 200), "GSM900 ARFCN 200 invalid");
    CHECK(arfcn_valid(Band::EGSM900, 1000), "EGSM900 ARFCN 1000 valid");

    // ---- 2. FIRE (224,184) parity ---------------------------------------------------------
    {
        uint8_t info[29] = {0};
        for (int i = 0; i < 23; ++i) info[i] = (uint8_t)(0x5a ^ (i * 7));  // arbitrary 184 bits
        fire_encode(info);
        CHECK(fire_check(info), "FIRE: encoded codeword passes check");
        info[3] ^= 0x08;  // flip one info bit
        CHECK(!fire_check(info), "FIRE: single-bit corruption rejected");
    }

    // ---- 3. Convolutional code + Viterbi (rate 1/2, K=5) ----------------------------------
    {
        uint8_t info[29] = {0};
        for (int i = 0; i < 23; ++i) info[i] = (uint8_t)(0xC3 + i);
        fire_encode(info);           // 184 info + 40 parity; bits 224..227 tail = 0
        uint8_t coded[57] = {0};
        conv_encode(info, 228, coded);
        uint8_t mother[456];
        for (int i = 0; i < 456; ++i) mother[i] = get_bit(coded, i);
        static uint8_t trace[228 * 16];
        uint8_t dec[29] = {0};
        int m = conv_viterbi(mother, 228, dec, trace);
        CHECK(m == 0 && memcmp(dec, info, 29) == 0, "Viterbi recovers clean stream (metric 0)");

        // inject 3 bit errors -> Viterbi should still correct
        mother[10] ^= 1; mother[100] ^= 1; mother[300] ^= 1;
        conv_viterbi(mother, 228, dec, trace);
        CHECK(memcmp(dec, info, 29) == 0, "Viterbi corrects 3 bit errors");
    }

    // ---- 4. Full xCCH block: encode -> 4 bursts -> decode ----------------------------------
    // Use the real SI3 L2 frame from simple_IMSI-catcher.py's find_cell() dump.
    const uint8_t si3_l2[23] = {
        0x49, 0x06, 0x1b, 0x61, 0x9d, 0x02, 0xf8, 0x02, 0x01, 0x9c, 0xc8, 0x03,
        0x1e, 0x53, 0xa5, 0x07, 0x79, 0x00, 0x00, 0x80, 0x01, 0x40, 0xdb};
    {
        uint8_t bursts[4][15];
        xcch_encode(si3_l2, bursts);
        uint8_t l2[23];
        CHECK(xcch_decode(bursts, l2), "xCCH: clean block decodes (FIRE ok)");
        CHECK(memcmp(l2, si3_l2, 23) == 0, "xCCH: recovered L2 == original SI3 frame");

        // corrupt a handful of bits across bursts -> Viterbi corrects, FIRE still ok
        bursts[0][2] ^= 0x01; bursts[1][5] ^= 0x02; bursts[2][8] ^= 0x04;
        CHECK(xcch_decode(bursts, l2) && memcmp(l2, si3_l2, 23) == 0,
              "xCCH: corrects a few bit errors and recovers SI3");

        // heavy corruption -> FIRE must reject (no false-positive frame)
        for (int b = 0; b < 4; ++b)
            for (int i = 0; i < 15; ++i) bursts[b][i] ^= 0xAA;
        CHECK(!xcch_decode(bursts, l2), "xCCH: heavily corrupted block rejected by FIRE");
    }

    // ---- 5. L3 find_cell(): SI3 -> MCC/MNC/LAC/CellID (France 208/20) ----------------------
    {
        uint8_t p[GSMTAP_PAYLOAD_LEN];
        build_gsmtap(p, si3_l2, GSMTAP_CHANNEL_BCCH, 62, 12345, 0, -63, 10);
        GsmParse r = gsm_l3_parse(p, GSMTAP_PAYLOAD_LEN);
        CHECK(r.has_cell, "SI3 parsed as cell");
        CHECK(r.cell.mcc == 208, "cell MCC == 208 (France)");
        CHECK(r.cell.mnc == 20, "cell MNC == 20 (Bouygues)");
        CHECK(r.cell.lac == 0x019c, "cell LAC == 0x019c (412)");
        CHECK(r.cell.cell_id == 0x619d, "cell CID == 0x619d (24989)");
        CHECK(mcc_country(r.cell.mcc) && std::strcmp(mcc_country(r.cell.mcc), "France") == 0,
              "MCC 208 -> France");
    }

    // ---- 6. L3 find_imsi(): Paging Request Type 1 -> IMSI ----------------------------------
    // IMSI 208150123456789 BCD (nibble-swapped, odd/IMSI type nibble 0x9): 29 80 51 10 32 54 76 98
    const uint8_t imsi_bcd[8] = {0x29, 0x80, 0x51, 0x10, 0x32, 0x54, 0x76, 0x98};
    {
        uint8_t l2[23];
        memset(l2, 0x2b, 23);
        l2[0] = 0x31; l2[1] = 0x06; l2[2] = 0x21; l2[3] = 0x00; l2[4] = 0x08;  // Paging1, len 8
        memcpy(&l2[5], imsi_bcd, 8);  // Mobile Identity 1 = IMSI at p[0x15]
        uint8_t p[GSMTAP_PAYLOAD_LEN];
        build_gsmtap(p, l2, GSMTAP_CHANNEL_CCCH, 62, 22, 0, -70, 8);
        GsmParse r = gsm_l3_parse(p, GSMTAP_PAYLOAD_LEN);
        CHECK(!r.has_cell && r.n_id == 1, "Paging1 yields exactly one identity");
        CHECK(r.id[0].type == GsmIdentity::IMSI, "Paging1 identity is IMSI");
        CHECK(std::strcmp(r.id[0].imsi, "208150123456789") == 0, "IMSI decoded == 208150123456789");
        CHECK(r.id[0].mcc == 208 && r.id[0].mnc == 15, "IMSI split MCC 208 / MNC 15");
    }

    // ---- 7. L3 find_imsi(): Paging Type 1 with two TMSIs -----------------------------------
    {
        uint8_t l2[23];
        memset(l2, 0x2b, 23);
        l2[0] = 0x41; l2[1] = 0x06; l2[2] = 0x21; l2[3] = 0x00; l2[4] = 0x05; l2[5] = 0xf4;
        l2[6] = 0xd9; l2[7] = 0x60; l2[8] = 0x54; l2[9] = 0x60;                 // TMSI1 at p[0x16]
        l2[10] = 0x17; l2[11] = 0x05; l2[12] = 0xf4;                            // IEI + type for id2
        l2[13] = 0x11; l2[14] = 0x22; l2[15] = 0x33; l2[16] = 0x44;             // TMSI2 at p[0x1D]
        uint8_t p[GSMTAP_PAYLOAD_LEN];
        build_gsmtap(p, l2, GSMTAP_CHANNEL_CCCH, 62, 23, 0, -70, 8);
        GsmParse r = gsm_l3_parse(p, GSMTAP_PAYLOAD_LEN);
        CHECK(r.n_id == 2 && r.id[0].type == GsmIdentity::TMSI && r.id[1].type == GsmIdentity::TMSI,
              "Paging1 yields two TMSIs");
        CHECK(r.id[0].tmsi == 0xd9605460 && r.id[1].tmsi == 0x11223344,
              "TMSI values 0xd9605460 / 0x11223344");
    }

    // ---- 8. L3 find_imsi(): Paging Type 2 -> TMSI, TMSI, IMSI ------------------------------
    {
        uint8_t l2[23];
        memset(l2, 0x2b, 23);
        l2[0] = 0x55; l2[1] = 0x06; l2[2] = 0x22; l2[3] = 0x00;                  // Paging2
        l2[4] = 0xaa; l2[5] = 0xbb; l2[6] = 0xcc; l2[7] = 0xdd;                  // TMSI1 at p[0x14]
        l2[8] = 0x11; l2[9] = 0x22; l2[10] = 0x33; l2[11] = 0x44;                // TMSI2 at p[0x18]
        l2[12] = 0x17; l2[13] = 0x08;                                           // IEI + len for id3
        memcpy(&l2[14], imsi_bcd, 8);                                           // IMSI at p[0x1E]
        uint8_t p[GSMTAP_PAYLOAD_LEN];
        build_gsmtap(p, l2, GSMTAP_CHANNEL_CCCH, 62, 24, 0, -70, 8);
        GsmParse r = gsm_l3_parse(p, GSMTAP_PAYLOAD_LEN);
        CHECK(r.n_id == 3, "Paging2 yields three identities");
        CHECK(r.id[0].tmsi == 0xaabbccdd && r.id[1].tmsi == 0x11223344, "Paging2 two TMSIs");
        CHECK(r.id[2].type == GsmIdentity::IMSI &&
              std::strcmp(r.id[2].imsi, "208150123456789") == 0, "Paging2 IMSI decoded");
    }

    // ---- 9. SCH: BSIC + frame number round-trip -------------------------------------------
    {
        uint32_t fn = 1000, t1 = fn / (26 * 51), t2 = fn % 26, t3 = fn % 51, t3p = (t3 - 1) / 10;
        uint8_t bsic = 0x2a;  // NCC 5, BCC 2
        uint8_t info25[4] = {0};
        put_bits(info25, 0, 6, bsic);
        put_bits(info25, 6, 11, t1);
        put_bits(info25, 17, 5, t2);
        put_bits(info25, 22, 3, t3p);
        uint8_t coded78[10];
        sch_encode(info25, coded78);
        uint8_t mother[78];
        for (int i = 0; i < 78; ++i) mother[i] = get_bit(coded78, i);
        SchInfo si = sch_decode(mother);
        CHECK(si.valid, "SCH decodes (CRC10 ok)");
        CHECK(si.bsic == bsic && si.ncc == 5 && si.bcc == 2, "SCH BSIC/NCC/BCC recovered");
        CHECK(si.fn == 1000, "SCH frame number == 1000");
        mother[40] ^= 1; mother[41] ^= 1; mother[42] ^= 1; mother[43] ^= 1;
        for (int i = 0; i < 20; ++i) mother[i] ^= 1;  // wreck it
        SchInfo bad = sch_decode(mother);
        CHECK(!bad.valid, "SCH rejects a wrecked burst");
    }

    // ---- 10. Tracks: dedup, count, TMSI vs IMSI -------------------------------------------
    {
        IdentityTable t;
        t.reset();
        GsmIdentity a; a.type = GsmIdentity::IMSI; std::strcpy(a.imsi, "208150123456789"); a.mcc = 208; a.mnc = 15;
        GsmIdentity b; b.type = GsmIdentity::TMSI; b.tmsi = 0xd9605460;
        CHECK(t.observe(a, 1.0f), "first IMSI is new");
        CHECK(!t.observe(a, 2.0f), "same IMSI again is not new");
        CHECK(t.observe(b, 3.0f), "first TMSI is new");
        CHECK(t.nb_imsi() == 1, "nb_IMSI counts distinct IMSIs (1), not TMSIs");
        CHECK(t.count() == 2, "table holds 2 records");
        t.purge_seconds = 5.0f;
        CHECK(t.purge(2000.0f) == 2, "stale records purged after timeout");
        CHECK(t.count() == 0, "table empty after purge");

        CellState cs;
        GsmCell c1{true, 208, 20, 0xff, 412, 24989};
        CHECK(cs.update(c1), "first cell is a change");
        CHECK(!cs.update(c1), "same cell is not a change");
        GsmCell c2{true, 208, 20, 0xff, 412, 24990};
        CHECK(cs.update(c2), "different CID is a change");
    }

    // ---- 12. IMSI not recounted/re-alerted after purge or eviction (fix) ------------------
    {
        IdentityTable t;
        t.reset();
        GsmIdentity a; a.type = GsmIdentity::IMSI; std::strcpy(a.imsi, "208150123456789");
        CHECK(t.observe(a, 1.0f) && t.nb_imsi() == 1, "IMSI first seen counts once");
        t.purge_seconds = 5.0f;
        t.purge(1000.0f);  // age A out of the active table
        CHECK(t.count() == 0, "active table emptied by purge");
        bool again = t.observe(a, 1001.0f);
        CHECK(!again && t.nb_imsi() == 1, "IMSI re-seen after purge is NOT recounted (nb stays 1)");
    }

    // ---- 11. TS0 51-multiframe schedule + xCCH block grouping -----------------------------
    {
        CHECK(chan_for_frame(0) == CHAN_FCCH && chan_for_frame(40) == CHAN_FCCH, "FCCH at 0/40");
        CHECK(chan_for_frame(1) == CHAN_SCH && chan_for_frame(41) == CHAN_SCH, "SCH at 1/41");
        CHECK(chan_for_frame(3) == CHAN_BCCH && chan_for_frame(5) == CHAN_BCCH, "BCCH at 2..5");
        CHECK(chan_for_frame(8) == CHAN_CCCH && chan_for_frame(49) == CHAN_CCCH, "CCCH frames");
        CHECK(chan_for_frame(50) == CHAN_IDLE, "frame 50 idle");
        int sf, pos; bool bcch;
        CHECK(xcch_block_of(4, sf, pos, bcch) && sf == 2 && pos == 2 && bcch, "frame 4 -> BCCH block pos 2");
        CHECK(xcch_block_of(14, sf, pos, bcch) && sf == 12 && pos == 2 && !bcch, "frame 14 -> CCCH block start 12 pos 2");
        CHECK(!xcch_block_of(1, sf, pos, bcch), "SCH frame is not an xCCH block frame");
    }

    std::printf("\n%s (%d failure%s)\n", g_fail ? "TESTS FAILED" : "ALL TESTS PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
