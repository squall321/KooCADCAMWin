#pragma once
// @lat: [[engine/skills#_keyway_table]]
//
// Central spec tables for keyway / key seat families:
//   - DIN 6885 Part A : parallel keys (rectangular cross-section)
//   - DIN 6886       : taper keys (1:100 taper)
//   - DIN 6888 / ANSI B17.2 : Woodruff keys (semi-circular arc)
//
// All key/keyway nominal dimensions in mm, tolerances per the standard
// (N9 normal-fit, P9 interference-fit).  Sources:
//   * DIN 6885-1 (1968) Table 1   — parallel key cross-sections
//   * DIN 6886   (1967) Table 1   — taper key cross-sections
//   * DIN 6888   (1956) Table 1   — Woodruff keys
//   * ANSI B17.2 (1967)           — Woodruff key supplemental data
//
// The shaft-size band (dia_min_mm, dia_max_mm) is INCLUSIVE on the upper
// bound, EXCLUSIVE on the lower bound — i.e. a 17 mm shaft uses the
// 12–17 band, an 18 mm shaft uses the 17–22 band.

#include <array>
#include <string>

namespace koocadcam::skill::keyway {

// ─────────────────────────────────────────────────────────────────────────
// DIN 6885 Part A — Parallel keys
// ─────────────────────────────────────────────────────────────────────────
struct Din6885ParallelBand {
    double      shaft_dia_min_mm;     // exclusive
    double      shaft_dia_max_mm;     // inclusive
    double      key_width_mm;         // b (key & keyway width)
    double      key_height_mm;        // h
    double      shaft_keyway_depth_mm; // t1 (in shaft, from OD)
    double      hub_keyway_depth_mm;  // t2 (in hub, from bore)
    const char* tolerance_grade;      // "N9" (normal) or "P9" (interference)
};

constexpr std::array<Din6885ParallelBand, 13> kDin6885Parallel{{
    //  >dia  ≤dia    b      h      t1     t2     grade
    {   6.0, 10.0,   3.0,   3.0,   1.8,   1.4,  "N9" },
    {  10.0, 12.0,   4.0,   4.0,   2.5,   1.8,  "N9" },
    {  12.0, 17.0,   5.0,   5.0,   3.0,   2.3,  "N9" },
    {  17.0, 22.0,   6.0,   6.0,   3.5,   2.8,  "N9" },
    {  22.0, 30.0,   8.0,   7.0,   4.0,   3.3,  "N9" },
    {  30.0, 38.0,  10.0,   8.0,   5.0,   3.3,  "N9" },
    {  38.0, 44.0,  12.0,   8.0,   5.0,   3.3,  "N9" },
    {  44.0, 50.0,  14.0,   9.0,   5.5,   3.8,  "N9" },
    {  50.0, 58.0,  16.0,  10.0,   6.0,   4.3,  "N9" },
    {  58.0, 65.0,  18.0,  11.0,   7.0,   4.4,  "N9" },
    {  65.0, 75.0,  20.0,  12.0,   7.5,   4.9,  "N9" },
    {  75.0, 85.0,  22.0,  14.0,   9.0,   5.4,  "N9" },
    {  85.0, 95.0,  25.0,  14.0,   9.0,   5.4,  "P9" },
}};

inline const Din6885ParallelBand* findDin6885Band(double shaft_dia_mm) {
    for (const auto& b : kDin6885Parallel) {
        if (shaft_dia_mm > b.shaft_dia_min_mm
            && shaft_dia_mm <= b.shaft_dia_max_mm) {
            return &b;
        }
    }
    return nullptr;
}

// ─────────────────────────────────────────────────────────────────────────
// DIN 6886 — Taper keys (1:100 taper from thick to thin end)
// ─────────────────────────────────────────────────────────────────────────
struct Din6886TaperBand {
    double      shaft_dia_min_mm;
    double      shaft_dia_max_mm;
    double      key_width_mm;                   // b
    double      key_thickness_at_thick_end_mm;  // h (at thick end)
    double      taper_per_100;                  // = 1.0 (1:100)
    double      shaft_keyway_depth_mm;          // t1
    double      hub_keyway_depth_mm;            // t2
};

constexpr std::array<Din6886TaperBand, 13> kDin6886Taper{{
    //  >dia  ≤dia    b     h    taper  t1    t2
    {   6.0, 10.0,   3.0,  3.0,  1.0,  1.8,  1.2 },
    {  10.0, 12.0,   4.0,  4.0,  1.0,  2.5,  1.5 },
    {  12.0, 17.0,   5.0,  5.0,  1.0,  3.0,  1.9 },
    {  17.0, 22.0,   6.0,  6.0,  1.0,  3.5,  2.3 },
    {  22.0, 30.0,   8.0,  7.0,  1.0,  4.0,  2.7 },
    {  30.0, 38.0,  10.0,  8.0,  1.0,  5.0,  2.7 },
    {  38.0, 44.0,  12.0,  8.0,  1.0,  5.0,  2.7 },
    {  44.0, 50.0,  14.0,  9.0,  1.0,  5.5,  3.1 },
    {  50.0, 58.0,  16.0, 10.0,  1.0,  6.0,  3.6 },
    {  58.0, 65.0,  18.0, 11.0,  1.0,  7.0,  3.6 },
    {  65.0, 75.0,  20.0, 12.0,  1.0,  7.5,  4.1 },
    {  75.0, 85.0,  22.0, 14.0,  1.0,  9.0,  4.6 },
    {  85.0, 95.0,  25.0, 14.0,  1.0,  9.0,  4.6 },
}};

inline const Din6886TaperBand* findDin6886Band(double shaft_dia_mm) {
    for (const auto& b : kDin6886Taper) {
        if (shaft_dia_mm > b.shaft_dia_min_mm
            && shaft_dia_mm <= b.shaft_dia_max_mm) {
            return &b;
        }
    }
    return nullptr;
}

// ─────────────────────────────────────────────────────────────────────────
// Woodruff keys — DIN 6888 / ANSI B17.2
// ─────────────────────────────────────────────────────────────────────────
struct WoodruffSpec {
    const char* key_id;                 // "404", "505", "606", ... (ANSI numbering)
    double      key_width_mm;           // b
    double      key_height_mm;          // h (full height above the chord)
    double      key_diameter_mm;        // d (arc diameter of the half-disc)
    double      shaft_keyway_depth_mm;  // depth of arc trough from OD
};

// ANSI B17.2 nominal key numbers — first digit(s) = width in 32nds inch
// (converted to mm), last two digits = diameter in 8ths inch.
// e.g. #808 → 1/4" × 1.0" → 6.35 mm wide × 25.4 mm diameter.
constexpr std::array<WoodruffSpec, 9> kWoodruff{{
    // id     b      h      d       depth
    { "404",  3.18,  6.35, 12.70,  4.85 },
    { "505",  3.97,  7.94, 15.88,  6.05 },
    { "606",  4.76,  9.53, 19.05,  7.25 },
    { "807",  6.35, 11.11, 22.22,  8.10 },
    { "808",  6.35, 12.70, 25.40,  9.65 },
    { "1008", 7.94, 12.70, 25.40,  9.25 },
    { "1009", 7.94, 14.29, 28.58, 10.85 },
    { "1011", 7.94, 17.46, 34.93, 13.85 },
    { "1212", 9.53, 19.05, 38.10, 15.05 },
}};

inline const WoodruffSpec* findWoodruff(const std::string& key_id) {
    for (const auto& w : kWoodruff) {
        if (key_id == w.key_id) return &w;
    }
    return nullptr;
}

}  // namespace koocadcam::skill::keyway
