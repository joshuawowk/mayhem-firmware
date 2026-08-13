/*
 * gsm_mcc.hpp — compact MCC→country + MNC-digit-count heuristic (portable, header-only).
 *
 * Copyright (C) 2026 (IMSI-catcher PortaPack port)  ·  GPLv2-or-later.
 *
 * The desktop tool ships mcc-mnc/mcc_codes.json (hundreds of KB of operator names). On-device
 * we keep only a compact country table for a friendly label; the full numeric MCC/MNC/IMSI is
 * always shown regardless, so an unknown code degrades gracefully. Expandable from SD later.
 */
#ifndef GSM_RX_GSM_MCC_HPP
#define GSM_RX_GSM_MCC_HPP

#include <cstdint>

namespace gsm_rx {

struct MccEntry {
    uint16_t mcc;
    const char* country;
};

// A representative subset; extend freely. Ranges collapsed to one label where a country owns
// several MCCs (e.g. USA 310-316).
static constexpr MccEntry MCC_TABLE[] = {
    {202, "Greece"},   {204, "Netherlands"}, {206, "Belgium"},  {208, "France"},
    {212, "Monaco"},   {213, "Andorra"},     {214, "Spain"},    {216, "Hungary"},
    {218, "Bosnia"},   {219, "Croatia"},     {222, "Italy"},    {226, "Romania"},
    {228, "Switzerland"}, {230, "Czechia"},  {231, "Slovakia"}, {232, "Austria"},
    {234, "UK"},       {235, "UK"},          {238, "Denmark"},  {240, "Sweden"},
    {242, "Norway"},   {244, "Finland"},     {246, "Lithuania"},{247, "Latvia"},
    {248, "Estonia"},  {250, "Russia"},      {255, "Ukraine"},  {257, "Belarus"},
    {259, "Moldova"},  {260, "Poland"},      {262, "Germany"},  {266, "Gibraltar"},
    {268, "Portugal"}, {270, "Luxembourg"},  {272, "Ireland"},  {274, "Iceland"},
    {276, "Albania"},  {278, "Malta"},       {280, "Cyprus"},   {284, "Bulgaria"},
    {286, "Turkey"},   {288, "Faroe Is."},   {293, "Slovenia"}, {294, "N.Macedonia"},
    {295, "Liechtenstein"}, {297, "Montenegro"},
    {302, "Canada"},   {310, "USA"},         {311, "USA"},      {312, "USA"},
    {313, "USA"},      {314, "USA"},         {315, "USA"},      {316, "USA"},
    {330, "PuertoRico"},{334, "Mexico"},     {338, "Jamaica"},  {340, "FrenchAntilles"},
    {404, "India"},    {405, "India"},       {410, "Pakistan"}, {413, "SriLanka"},
    {414, "Myanmar"},  {415, "Lebanon"},     {416, "Jordan"},   {418, "Iraq"},
    {419, "Kuwait"},   {420, "SaudiArabia"}, {424, "UAE"},      {425, "Israel"},
    {432, "Iran"},     {440, "Japan"},       {441, "Japan"},    {450, "SouthKorea"},
    {452, "Vietnam"},  {454, "HongKong"},    {455, "Macau"},    {460, "China"},
    {466, "Taiwan"},   {470, "Bangladesh"},  {502, "Malaysia"}, {505, "Australia"},
    {510, "Indonesia"},{515, "Philippines"}, {520, "Thailand"}, {525, "Singapore"},
    {530, "NewZealand"},{602, "Egypt"},      {605, "Tunisia"},  {607, "Gambia"},
    {655, "SouthAfrica"},{724, "Brazil"},    {722, "Argentina"},{730, "Chile"},
    {732, "Colombia"}, {734, "Venezuela"},   {748, "Uruguay"},
};

inline const char* mcc_country(uint16_t mcc) {
    for (const auto& e : MCC_TABLE)
        if (e.mcc == mcc) return e.country;
    return nullptr;
}

// Countries that use 3-digit MNCs (North America chiefly). The desktop tool decides via table
// lookup; this heuristic covers the common cases so a 3-digit MNC displays sensibly.
inline bool mnc_is_3digit(uint16_t mcc) {
    if (mcc >= 310 && mcc <= 316) return true;  // USA
    switch (mcc) {
        case 302:  // Canada
        case 334:  // Mexico
        case 732:  // Colombia
        case 330:  // Puerto Rico
            return true;
        default:
            return false;
    }
}

}  // namespace gsm_rx

#endif  // GSM_RX_GSM_MCC_HPP
