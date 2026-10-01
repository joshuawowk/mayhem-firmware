/*
 * gsm_offline_decode.cpp — host harness that runs the REAL gsm_rx demod + decode pipeline
 * (a faithful float port of proc_gsm's GMSK/Gardner/SCH-sync framing + the portable
 * gsm_channel/gsm_l3 cores) on either a synthetic GSM TS0 signal (--sim, default) or a real
 * HackRF IQ capture (--iq <file> [--rate Hz]).
 *
 * GPLv2-or-later. Build:
 *   g++ -std=c++17 -O2 -o /tmp/gsm_dec gsm_offline_decode.cpp && /tmp/gsm_dec --sim
 *
 * Purpose: validate the on-air-unverifiable half of the port (FCCH/SCH lock + GMSK burst
 * decode) against a modulated signal, and decode real captures once hardware tooling is up.
 */
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <complex>
#include <vector>
#include <string>
#include <functional>

#include "gsm_bands.hpp"
#include "gsm_channel.hpp"
#include "gsm_gsmtap.hpp"
#include "gsm_l3.hpp"
#include "gsm_tracks.hpp"
#include "gsm_mcc.hpp"

using namespace gsm_rx;
using cf = std::complex<float>;

// ---- GMSK modulator (matches the differential detector used by the demod) ------------------
static std::vector<double> gpulse(int sps, double bt, int span_syms) {
    // Gaussian-filtered rectangular frequency pulse, normalized so its samples sum to 1
    // (=> each bit contributes ±pi/2 total phase).
    const double sigma = std::sqrt(std::log(2.0)) / (2.0 * M_PI * bt);  // in symbol periods
    const int N = span_syms * sps;
    std::vector<double> g(N, 0.0);
    double sum = 0.0;
    for (int n = 0; n < N; ++n) {
        double t = (double)(n - N / 2) / sps;  // in symbols, centered
        // convolution of rect([-0.5,0.5]) with gaussian -> difference of Q-functions
        double a = (t + 0.5) / (sigma * std::sqrt(2.0));
        double b = (t - 0.5) / (sigma * std::sqrt(2.0));
        g[n] = 0.5 * (std::erf(a) - std::erf(b));
        sum += g[n];
    }
    for (auto& v : g) v /= sum;
    return g;
}

static std::vector<cf> gmsk_mod(const std::vector<uint8_t>& bits, int sps, double amp) {
    const auto g = gpulse(sps, 0.3, 4);
    const int gN = (int)g.size();
    const int N = (int)bits.size() * sps + gN;
    std::vector<double> freq(N, 0.0);
    for (size_t i = 0; i < bits.size(); ++i) {
        double a = bits[i] ? 1.0 : -1.0;   // NRZ
        int base = (int)i * sps;
        for (int k = 0; k < gN; ++k) freq[base + k] += a * (M_PI / 2.0) * g[k];
    }
    std::vector<cf> out(N);
    double phase = 0.0;
    for (int n = 0; n < N; ++n) {
        phase += freq[n];
        out[n] = cf((float)(amp * std::cos(phase)), (float)(amp * std::sin(phase)));
    }
    return out;
}

// ---- Demod: faithful float port of proc_gsm ------------------------------------------------
struct Burst { uint8_t payload[15]; uint8_t type; uint8_t tsc; uint8_t errors; };

class Demod {
   public:
    std::function<void(const Burst&)> on_burst;
    explicit Demod(double sps) {
        symbol_phase_inc = (uint32_t)(4294967296.0 / sps);
        inc_nom = symbol_phase_inc;
    }
    void push(cf s) {
        uint32_t old = symbol_phase;
        symbol_phase += symbol_phase_inc;
        if (old < 0x80000000u && symbol_phase >= 0x80000000u) mid = s;
        if (symbol_phase < old) {  // full-symbol
            prev_prompt = prompt; prompt = s;
            double err = mid.real() * (prompt.real() - prev_prompt.real()) +
                         mid.imag() * (prompt.imag() - prev_prompt.imag());
            int64_t d = (int64_t)(err) >> 16;
            int64_t v = (int64_t)symbol_phase_inc + d;
            int64_t lo = inc_nom - inc_nom / 50, hi = inc_nom + inc_nom / 50;  // ±2%
            if (v < lo) v = lo; if (v > hi) v = hi;
            symbol_phase_inc = (uint32_t)v;
            demod_bit(prompt);
        }
    }
   private:
    static constexpr uint32_t BURST_BITS = 148, SCH_TRAIN_POS = 42, SCH_TRAIN_BITS = 64;
    static constexpr uint32_t TSC_POS = 61, TSC_BITS = 26, FRAME_SYMBOLS = 1250;
    static constexpr uint64_t SCH_TRAIN = 0xB962083E6D45761BULL;
    static constexpr uint32_t TSC_MASK = 0x03FFFFFF;
    static constexpr uint32_t TSC[8] = {
        0b00100101110000100010010111, 0b00101101110111100010110111,
        0b01000011101110100100001110, 0b01000111101101000100011110,
        0b00011010111001000001101011, 0b01001110101100000100111010,
        0b10100111110110001010011111, 0b11101111000100101110111100};
    static constexpr uint32_t SCH_SYNC_THRESH = 8, TSC_THRESH = 5, FCCH_MAX_TRANS = 10;

    uint32_t symbol_phase{0}, symbol_phase_inc, inc_nom;
    cf prompt{0, 0}, prev_prompt{0, 0}, mid{0, 0}, prev_symbol{0, 0};
    uint64_t sync_register{0}, bit_count{0}, next_burst_start{0};
    uint8_t bit_history[256] = {0};
    bool locked{false};

    uint8_t hbit(uint64_t a) const { size_t p = a % 2048; return (bit_history[p >> 3] >> (7 - (p & 7))) & 1; }

    void demod_bit(cf cur) {
        double dd = (double)prev_symbol.real() * cur.imag() - (double)prev_symbol.imag() * cur.real();
        prev_symbol = cur;
        on_new_bit(dd > 0 ? 1 : 0);
    }
    void on_new_bit(uint8_t bit) {
        size_t idx = bit_count % 2048;
        if ((idx & 7) == 0) bit_history[idx >> 3] = 0;
        bit_history[idx >> 3] |= (uint8_t)(bit << (7 - (idx & 7)));
        bit_count++;
        sync_register = (sync_register << 1) | bit;
        if (bit_count >= SCH_TRAIN_BITS) {
            uint32_t ep = __builtin_popcountll(sync_register ^ SCH_TRAIN);
            uint32_t en = __builtin_popcountll(sync_register ^ ~SCH_TRAIN);
            if (ep <= SCH_SYNC_THRESH || en <= SCH_SYNC_THRESH) {
                next_burst_start = (bit_count - 1) - (SCH_TRAIN_POS + SCH_TRAIN_BITS - 1);
                locked = true;
            }
        }
        if (locked && bit_count >= next_burst_start + BURST_BITS) {
            extract();
            next_burst_start += FRAME_SYMBOLS;
        }
    }
    void extract() {
        uint64_t bs = next_burst_start;
        uint64_t tr = 0;
        for (uint32_t i = 0; i < SCH_TRAIN_BITS; i++) tr = (tr << 1) | hbit(bs + SCH_TRAIN_POS + i);
        uint32_t sp = __builtin_popcountll(tr ^ SCH_TRAIN), sn = __builtin_popcountll(tr ^ ~SCH_TRAIN);
        uint32_t serr = std::min(sp, sn); bool sinv = sn < sp;
        Burst b{};
        if (serr <= SCH_SYNC_THRESH) {
            uint32_t o = 0; auto em = [&](uint32_t i){ uint8_t v = hbit(bs + i) ^ (sinv?1:0); b.payload[o>>3] |= v<<(7-(o&7)); o++; };
            for (uint32_t i = 3; i <= 41; i++) em(i);
            for (uint32_t i = 106; i <= 144; i++) em(i);
            b.type = 1; b.errors = serr; if (on_burst) on_burst(b); return;
        }
        uint32_t trans = 0; uint8_t pb = hbit(bs + 3);
        for (uint32_t i = 4; i <= 144; i++) { uint8_t cb = hbit(bs + i); trans += cb ^ pb; pb = cb; }
        if (trans < FCCH_MAX_TRANS) { b.type = 2; if (on_burst) on_burst(b); return; }
        uint32_t win = 0; for (uint32_t i = 0; i < TSC_BITS; i++) win = (win << 1) | hbit(bs + TSC_POS + i);
        uint32_t bk = 0, be = TSC_BITS + 1; bool bi = false;
        for (uint32_t k = 0; k < 8; k++) { uint32_t e = std::min(__builtin_popcount(win ^ TSC[k]), __builtin_popcount(win ^ (~TSC[k] & TSC_MASK))); if (e < be) { be = e; bk = k; bi = __builtin_popcount(win ^ (~TSC[k] & TSC_MASK)) < __builtin_popcount(win ^ TSC[k]); } }
        if (be > TSC_THRESH) { b.type = 3; b.errors = be; if (on_burst) on_burst(b); return; }
        uint32_t o = 0; auto em = [&](uint32_t i){ uint8_t v = hbit(bs + i) ^ (bi?1:0); b.payload[o>>3] |= v<<(7-(o&7)); o++; };
        for (uint32_t i = 3; i <= 59; i++) em(i);
        for (uint32_t i = 88; i <= 144; i++) em(i);
        b.type = 0; b.tsc = (uint8_t)bk; b.errors = (uint8_t)be; if (on_burst) on_burst(b);
    }
};

// ---- M0 assembler: mirrors ui_gsm_rx block grouping + decode -------------------------------
struct Assembler {
    int cur_mf = -1; uint32_t abs_fn = 0;
    uint8_t block[4][15]; uint32_t block_fn = 0; bool block_active = false; uint8_t mask = 0; bool bcch = false;
    IdentityTable ids; CellState cell;
    int sch_ok = 0, blocks_ok = 0, fcch = 0, normals = 0;

    void feed(const Burst& m) {
        if (cur_mf >= 0) { cur_mf = (cur_mf + 1) % 51; abs_fn++; }
        if (m.type == 1) {
            uint8_t mother[78]; for (int i = 0; i < 78; i++) mother[i] = get_bit(m.payload, i);
            SchInfo si = sch_decode(mother);
            if (si.valid) { cur_mf = (int)(si.fn % 51); abs_fn = si.fn; sch_ok++;
                std::printf("  [SCH]  lock BSIC=%u FN=%u -> mf=%d\n", si.bsic, si.fn, cur_mf); }
        } else if (m.type == 0) {
            normals++;
            if (cur_mf < 0) return;
            int s, p; bool bc; if (!xcch_block_of(cur_mf, s, p, bc)) return;
            uint32_t bfn = abs_fn - (uint32_t)p;
            if (!block_active || bfn != block_fn) { block_fn = bfn; block_active = true; mask = 0; bcch = bc; }
            for (int i = 0; i < 15; i++) block[p][i] = m.payload[i];
            mask |= (1 << p);
            if (mask == 0x0F) { decode_block(); mask = 0; block_active = false; }
        } else if (m.type == 2) fcch++;
    }
    void decode_block() {
        uint8_t l2[23]; if (!xcch_decode(block, l2)) return; blocks_ok++;
        uint8_t g[GSMTAP_PAYLOAD_LEN];
        build_gsmtap(g, l2, bcch ? GSMTAP_CHANNEL_BCCH : GSMTAP_CHANNEL_CCCH, 62, abs_fn, 0, 0, 0);
        GsmParse r = gsm_l3_parse(g, GSMTAP_PAYLOAD_LEN);
        if (r.has_cell && cell.update(r.cell))
            std::printf("  [CELL] MCC=%u MNC=%02u LAC=0x%04x CID=0x%04x %s\n",
                        r.cell.mcc, r.cell.mnc, r.cell.lac, r.cell.cell_id,
                        mcc_country(r.cell.mcc) ? mcc_country(r.cell.mcc) : "");
        for (int i = 0; i < r.n_id; i++) {
            bool nu = ids.observe(r.id[i], 0.0f);
            if (r.id[i].type == GsmIdentity::IMSI && nu)
                std::printf("  [IMSI] %s (MCC=%u MNC=%u)  #%u\n", r.id[i].imsi, r.id[i].mcc, r.id[i].mnc, ids.nb_imsi());
            else if (r.id[i].type == GsmIdentity::TMSI && nu)
                std::printf("  [TMSI] 0x%08x\n", r.id[i].tmsi);
        }
    }
};

// ---- synthetic GSM TS0 builder -------------------------------------------------------------
static void put_bits_msb(uint8_t* p, int off, int n, uint32_t v) {
    for (int i = 0; i < n; ++i) set_bit(p, off + i, (v >> (n - 1 - i)) & 1);
}
static void place(std::vector<uint8_t>& stream, const uint8_t burst[19]) {  // 148 bits
    for (int i = 0; i < 148; ++i) stream.push_back(get_bit(burst, i));
    for (int i = 0; i < 1250 - 148; ++i) stream.push_back(0);  // filler to next TS0
}
static void build_sch_burst(uint8_t out[19], uint32_t fn, uint8_t bsic) {
    memset(out, 0, 19);
    uint32_t t1 = fn / (26 * 51), t2 = fn % 26, t3 = fn % 51, t3p = (t3 - 1) / 10;
    uint8_t info[4] = {0};
    put_bits_msb(info, 0, 6, bsic); put_bits_msb(info, 6, 11, t1);
    put_bits_msb(info, 17, 5, t2); put_bits_msb(info, 22, 3, t3p);
    uint8_t coded[10]; sch_encode(info, coded);
    for (int i = 0; i < 39; i++) set_bit(out, 3 + i, get_bit(coded, i));
    for (int i = 0; i < 64; i++) set_bit(out, 42 + i, (0xB962083E6D45761BULL >> (63 - i)) & 1);
    for (int i = 0; i < 39; i++) set_bit(out, 106 + i, get_bit(coded, 39 + i));
}
static void build_normal_burst(uint8_t out[19], const uint8_t burst114[15]) {
    memset(out, 0, 19);
    static const uint32_t TSC0 = 0b00100101110000100010010111;
    for (int i = 0; i < 57; i++) set_bit(out, 3 + i, get_bit(burst114, i));
    for (int i = 0; i < 26; i++) set_bit(out, 61 + i, (TSC0 >> (25 - i)) & 1);
    for (int i = 0; i < 57; i++) set_bit(out, 88 + i, get_bit(burst114, 57 + i));
}

static std::vector<uint8_t> build_sim_stream() {
    // one C0/TS0 multiframe head: FCCH(0) SCH(1) BCCH SI3(2-5) CCCH paging(6-9) FCCH(10) SCH(11)
    const uint8_t si3[23] = {0x49,0x06,0x1b,0x61,0x9d,0x02,0xf8,0x02,0x01,0x9c,0xc8,0x03,0x1e,0x53,0xa5,0x07,0x79,0x00,0x00,0x80,0x01,0x40,0xdb};
    const uint8_t imsi_bcd[8] = {0x29,0x80,0x51,0x10,0x32,0x54,0x76,0x98};  // IMSI 208150123456789
    uint8_t paging[23]; memset(paging, 0x2b, 23);
    paging[0]=0x31; paging[1]=0x06; paging[2]=0x21; paging[3]=0x00; paging[4]=0x08; memcpy(&paging[5], imsi_bcd, 8);

    uint8_t bcch_b[4][15], ccch_b[4][15];
    xcch_encode(si3, bcch_b); xcch_encode(paging, ccch_b);

    std::vector<uint8_t> s;
    uint8_t fcch[19]; memset(fcch, 0, 19);         // all-zero -> tone
    uint8_t schb[19]; build_sch_burst(schb, 1, 0x2a);   // FN=1 -> mf 1
    uint8_t dummy[19]; uint8_t z114[15] = {0}; build_normal_burst(dummy, z114);

    for (int mf = 0; mf < 12; ++mf) {
        uint8_t b[19];
        if (mf % 10 == 0) { place(s, fcch); }
        else if (mf % 10 == 1) { build_sch_burst(b, (uint32_t)mf, 0x2a); place(s, b); }
        else if (mf >= 2 && mf <= 5) { build_normal_burst(b, bcch_b[mf - 2]); place(s, b); }
        else if (mf >= 6 && mf <= 9) { build_normal_burst(b, ccch_b[mf - 6]); place(s, b); }
        else place(s, dummy);
    }
    return s;
}

int main(int argc, char** argv) {
    std::string iq_file; double rate = 3072000; int sps_sim = 8;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--iq" && i + 1 < argc) iq_file = argv[++i];
        else if (a == "--rate" && i + 1 < argc) rate = atof(argv[++i]);
        else if (a == "--sps" && i + 1 < argc) sps_sim = atoi(argv[++i]);
    }

    // box-filter decimate the capture toward the device's ~2.84 samples/symbol channel rate.
    int dec = iq_file.empty() ? 1 : std::max(1, (int)std::llround(rate / (270833.33 * 2.84)));
    double sps_eff = iq_file.empty() ? sps_sim : (rate / dec) / 270833.33;

    Assembler asm_;
    Demod dm(sps_eff);
    dm.on_burst = [&](const Burst& b) { asm_.feed(b); };

    if (iq_file.empty()) {
        std::printf("== SIM: GMSK TS0 (SI3 208/20 + Paging IMSI 208150123456789), sps=%d ==\n", sps_sim);
        auto bits = build_sim_stream();
        auto iq = gmsk_mod(bits, sps_sim, 4000.0);
        for (auto& s : iq) dm.push(s);
    } else {
        std::printf("== IQ decode: %s @ %.0f Hz, /%d -> %.0f Hz (%.2f samples/symbol) ==\n",
                    iq_file.c_str(), rate, dec, rate / dec, sps_eff);
        FILE* f = fopen(iq_file.c_str(), "rb");
        if (!f) { std::printf("cannot open %s\n", iq_file.c_str()); return 1; }
        int8_t buf[8192]; long acc_i = 0, acc_q = 0; int cnt = 0; size_t n; double p2 = 0; uint64_t ns = 0;
        while ((n = fread(buf, 2, sizeof(buf) / 2, f)) > 0) {
            for (size_t k = 0; k < n; ++k) {
                p2 += (double)buf[2 * k] * buf[2 * k] + (double)buf[2 * k + 1] * buf[2 * k + 1]; ns++;
                acc_i += buf[2 * k]; acc_q += buf[2 * k + 1];
                if (++cnt == dec) { dm.push(cf((float)acc_i / dec * 32.0f, (float)acc_q / dec * 32.0f)); acc_i = acc_q = 0; cnt = 0; }
            }
        }
        fclose(f);
        std::printf("   capture RMS = %.1f (of 127 full-scale, %llu samples)\n",
                    std::sqrt(p2 / (ns ? ns : 1)), (unsigned long long)ns);
    }

    std::printf("\n-- summary: SCH-lock=%d FCCH=%d normals=%d blocks-decoded=%d cells=%d IMSI=%u --\n",
                asm_.sch_ok, asm_.fcch, asm_.normals, asm_.blocks_ok, asm_.cell.valid ? 1 : 0, asm_.ids.nb_imsi());
    bool sim = iq_file.empty();
    if (sim) {
        bool ok = asm_.cell.valid && asm_.cell.cell.mcc == 208 && asm_.cell.cell.cell_id == 0x619d && asm_.ids.nb_imsi() >= 1;
        std::printf("%s\n", ok ? "SIM RESULT: PASS (locked, decoded SI3 cell + IMSI end-to-end)"
                              : "SIM RESULT: FAIL (see above)");
        return ok ? 0 : 1;
    }
    return 0;
}
