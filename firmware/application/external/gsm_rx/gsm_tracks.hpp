/*
 * gsm_tracks.hpp — on-device identity + cell tables (portable, header-only).
 *
 * Copyright (C) 2026 (IMSI-catcher PortaPack port)  ·  GPLv2-or-later.
 *
 * Device-side replacement for the desktop tracker class (simple_IMSI-catcher.py): dedups
 * observed IMSIs/TMSIs into a live table, counts distinct IMSIs (nb_IMSI), ages entries out
 * after a purge timer (default 10 min like the original), and holds the current cell. Fixed
 * arrays, no allocation — safe for the M0 and host-testable.
 */
#ifndef GSM_RX_GSM_TRACKS_HPP
#define GSM_RX_GSM_TRACKS_HPP

#include <cstdint>
#include <cstring>
#include "gsm_l3.hpp"

namespace gsm_rx {

static constexpr float GSM_PURGE_S = 600.0f;  // 10 minutes (tracker.purgeTimer)

struct IdentityRecord {
    bool used = false;
    GsmIdentity::Type type = GsmIdentity::NONE;
    char imsi[16] = {0};
    uint32_t tmsi = 0;
    uint16_t mcc = 0;
    uint16_t mnc = 0;
    float first_seen_s = 0;
    float last_seen_s = 0;
    uint32_t count = 0;      // observations
    uint32_t index = 0;      // 1-based order of first appearance (like nb_IMSI)
};

class IdentityTable {
   public:
    static constexpr int MAX = 32;
    float purge_seconds = GSM_PURGE_S;

    void reset() {
        for (int i = 0; i < MAX; ++i) rec_[i].used = false;
        nb_imsi_ = 0;
        seen_n_ = 0;
    }

    // Insert/refresh an identity. Returns true if it is newly seen (first appearance).
    bool observe(const GsmIdentity& id, float now_s) {
        if (id.type == GsmIdentity::NONE) return false;
        int s = find(id);
        if (s >= 0) {
            rec_[s].last_seen_s = now_s;
            ++rec_[s].count;
            return false;
        }
        int slot = free_slot();
        if (slot < 0) slot = oldest_slot();
        IdentityRecord& r = rec_[slot];
        r = IdentityRecord{};
        r.used = true;
        r.type = id.type;
        r.mcc = id.mcc;
        r.mnc = id.mnc;
        r.first_seen_s = now_s;
        r.last_seen_s = now_s;
        r.count = 1;
        bool newly = true;
        if (id.type == GsmIdentity::IMSI) {
            int j = 0;
            for (; j < (int)sizeof(r.imsi) - 1 && id.imsi[j]; ++j) r.imsi[j] = id.imsi[j];
            r.imsi[j] = '\0';
            // Count distinct IMSIs against a never-evicted seen-set (mirrors the desktop
            // tracker's self.imsis vs self.imsistate), so an IMSI re-observed after LRU
            // eviction/purge is neither double-counted nor re-alerted.
            newly = mark_seen_imsi(r.imsi);
            r.index = newly ? ++nb_imsi_ : 0;
        } else {
            r.tmsi = id.tmsi;
            r.index = 0;  // TMSIs are not counted toward nb_IMSI
        }
        return newly;
    }

    int purge(float now_s) {
        int n = 0;
        for (int i = 0; i < MAX; ++i)
            if (rec_[i].used && (now_s - rec_[i].last_seen_s) > purge_seconds) {
                rec_[i].used = false;
                ++n;
            }
        return n;
    }

    uint32_t nb_imsi() const { return nb_imsi_; }
    int count() const {
        int n = 0;
        for (int i = 0; i < MAX; ++i)
            if (rec_[i].used) ++n;
        return n;
    }
    const IdentityRecord& at(int i) const { return rec_[i]; }
    int capacity() const { return MAX; }

   private:
    IdentityRecord rec_[MAX] = {};
    uint32_t nb_imsi_ = 0;

    // ever-seen IMSIs (numeric keys), independent of the purgeable active table — gates the
    // distinct-IMSI count so eviction/re-observation cannot inflate it.
    static constexpr int SEEN_MAX = 96;
    uint64_t seen_[SEEN_MAX] = {};
    int seen_n_ = 0;

    static uint64_t imsi_key(const char* s) {  // up to 15 digits fits in uint64_t
        uint64_t k = 0;
        for (int i = 0; s[i]; ++i) k = k * 10 + (uint64_t)(s[i] - '0');
        return k;
    }
    bool mark_seen_imsi(const char* imsi) {  // true iff this IMSI had not been seen before
        uint64_t k = imsi_key(imsi);
        for (int i = 0; i < seen_n_; ++i)
            if (seen_[i] == k) return false;
        if (seen_n_ < SEEN_MAX) seen_[seen_n_++] = k;
        return true;
    }

    int find(const GsmIdentity& id) const {
        for (int i = 0; i < MAX; ++i) {
            if (!rec_[i].used || rec_[i].type != id.type) continue;
            if (id.type == GsmIdentity::IMSI) {
                if (std::strncmp(rec_[i].imsi, id.imsi, sizeof(rec_[i].imsi)) == 0) return i;
            } else if (rec_[i].tmsi == id.tmsi) {
                return i;
            }
        }
        return -1;
    }
    int free_slot() const {
        for (int i = 0; i < MAX; ++i)
            if (!rec_[i].used) return i;
        return -1;
    }
    int oldest_slot() const {
        int best = 0;
        float oldest = rec_[0].last_seen_s;
        for (int i = 1; i < MAX; ++i)
            if (rec_[i].last_seen_s < oldest) { oldest = rec_[i].last_seen_s; best = i; }
        return best;
    }
};

// Current-cell holder (last SI3 decoded). Also flags a change so the UI can log it.
struct CellState {
    GsmCell cell;
    bool valid = false;

    // Returns true if the cell identity changed (new MCC/MNC/LAC/CID).
    bool update(const GsmCell& c) {
        bool changed = !valid || c.mcc != cell.mcc || c.mnc != cell.mnc ||
                       c.lac != cell.lac || c.cell_id != cell.cell_id;
        cell = c;
        valid = true;
        return changed;
    }
};

}  // namespace gsm_rx

#endif  // GSM_RX_GSM_TRACKS_HPP
