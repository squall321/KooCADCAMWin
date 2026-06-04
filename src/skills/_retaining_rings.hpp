#pragma once
// @lat: [[engine/skills#retaining_rings_table]]
//
// Central spec tables for retaining-ring and spring-washer skills.
// One source of truth for:
//   - DIN 471  (external / shaft retaining ring)
//   - DIN 472  (internal / bore  retaining ring)
//   - DIN 6799 (E-clip / axial retaining ring)
//   - DIN 127  (split-ring spring lock washer)
//
// All numeric values are in millimetres.
// Sources:
//   - DIN 471-2017,  DIN 472-2017
//   - DIN 6799-2017 (E-clip)
//   - DIN 127-1   (curved spring lock washer)
//   - ISO 5417 / "spiral retaining ring" geometry is computed from
//     shaft_dia and ring_thickness × turn_count at apply-time, so no table
//     entry is needed for DIN 5417.

#include <array>
#include <string>

namespace koocadcam::skill::retaining_rings {

// ── DIN 471 — External (shaft) retaining ring ───────────────────────────────
struct Din471Spec {
    const char* size_key;          // "6mm" .. "100mm"
    double shaft_dia_mm;           // nominal shaft outer Ø
    double groove_dia_mm;          // = shaft − 2·depth
    double groove_width_mm;
    double groove_depth_mm;
    double ring_thickness_mm;
};

constexpr std::array<Din471Spec, 14> kDin471 {{
    // key        shaft  groove   width  depth  thk
    { "6mm",      6.0,   5.7,     0.8,   0.15,  0.7 },
    { "8mm",      8.0,   7.6,     0.9,   0.2,   0.8 },
    { "10mm",    10.0,   9.6,     1.1,   0.2,   1.0 },
    { "12mm",    12.0,  11.5,     1.1,   0.25,  1.0 },
    { "14mm",    14.0,  13.4,     1.1,   0.3,   1.0 },
    { "16mm",    16.0,  15.2,     1.1,   0.4,   1.0 },
    { "20mm",    20.0,  19.0,     1.3,   0.5,   1.2 },
    { "25mm",    25.0,  23.9,     1.3,   0.55,  1.2 },
    { "30mm",    30.0,  28.6,     1.6,   0.7,   1.5 },
    { "40mm",    40.0,  38.0,     1.85,  1.0,   1.75},
    { "50mm",    50.0,  47.0,     2.15,  1.5,   2.0 },
    { "60mm",    60.0,  57.0,     2.15,  1.5,   2.0 },
    { "75mm",    75.0,  70.5,     2.65,  2.25,  2.5 },
    { "100mm",  100.0,  93.0,     3.15,  3.5,   3.0 },
}};

inline const Din471Spec* findDin471(const std::string& size_key) {
    for (const auto& s : kDin471) if (size_key == s.size_key) return &s;
    return nullptr;
}

// ── DIN 472 — Internal (bore) retaining ring ────────────────────────────────
struct Din472Spec {
    const char* size_key;          // "8mm" .. "100mm"
    double bore_dia_mm;            // nominal bore Ø
    double groove_dia_mm;          // = bore + 2·depth
    double groove_width_mm;
    double groove_depth_mm;
    double ring_thickness_mm;
};

constexpr std::array<Din472Spec, 13> kDin472 {{
    // key        bore   groove   width  depth  thk
    { "8mm",      8.0,    8.4,    0.9,   0.2,   0.8 },
    { "10mm",    10.0,   10.4,    1.1,   0.2,   1.0 },
    { "12mm",    12.0,   12.5,    1.1,   0.25,  1.0 },
    { "14mm",    14.0,   14.6,    1.1,   0.3,   1.0 },
    { "16mm",    16.0,   16.8,    1.1,   0.4,   1.0 },
    { "20mm",    20.0,   21.0,    1.3,   0.5,   1.2 },
    { "25mm",    25.0,   26.2,    1.3,   0.6,   1.2 },
    { "30mm",    30.0,   31.4,    1.6,   0.7,   1.5 },
    { "40mm",    40.0,   42.0,    1.85,  1.0,   1.75},
    { "50mm",    50.0,   53.0,    2.15,  1.5,   2.0 },
    { "60mm",    60.0,   63.0,    2.15,  1.5,   2.0 },
    { "75mm",    75.0,   79.5,    2.65,  2.25,  2.5 },
    { "100mm",  100.0,  107.0,    3.15,  3.5,   3.0 },
}};

inline const Din472Spec* findDin472(const std::string& size_key) {
    for (const auto& s : kDin472) if (size_key == s.size_key) return &s;
    return nullptr;
}

// ── DIN 6799 — E-clip / axial retaining ring ────────────────────────────────
//
// E-clip slides into a radial groove from one side.  groove_axial_position_mm
// is the recommended distance from the end of the shaft to the centre of the
// groove (typical use).
struct Din6799Spec {
    const char* size_key;            // "1.2mm" .. "9mm"  (nominal slot ID)
    double shaft_dia_mm;             // nominal groove-bottom shaft Ø before cut
    double groove_dia_mm;            // = shaft − 2·depth  (depth derived)
    double groove_width_mm;
    double groove_axial_position_mm; // typical centre-to-end recess
};

constexpr std::array<Din6799Spec, 10> kDin6799 {{
    // key       shaft   groove  width  axial_pos
    { "1.2mm",   1.6,    1.2,    0.4,   1.0 },
    { "1.5mm",   2.0,    1.5,    0.4,   1.0 },
    { "1.9mm",   2.5,    1.9,    0.4,   1.2 },
    { "2.3mm",   3.0,    2.3,    0.5,   1.5 },
    { "3.2mm",   4.0,    3.2,    0.6,   2.0 },
    { "4mm",     5.0,    4.0,    0.7,   2.5 },
    { "5mm",     6.0,    5.0,    0.8,   3.0 },
    { "6mm",     7.0,    6.0,    1.0,   3.5 },
    { "7mm",     8.0,    7.0,    1.1,   4.0 },
    { "9mm",    10.0,    9.0,    1.3,   5.0 },
}};

inline const Din6799Spec* findDin6799(const std::string& size_key) {
    for (const auto& s : kDin6799) if (size_key == s.size_key) return &s;
    return nullptr;
}

// ── DIN 127 — Split-ring spring lock washer ─────────────────────────────────
//
// Form B (rectangular profile).  thickness_mm is the wire thickness;
// gap_height_mm is the free-state axial separation of the split ends.
struct Din127Spec {
    const char* size_key;          // "M3" .. "M20"
    double inner_dia_mm;
    double outer_dia_mm;
    double thickness_mm;
    double gap_height_mm;
};

constexpr std::array<Din127Spec, 9> kDin127 {{
    // key   ID     OD     thk    gap
    { "M3",  3.1,   6.2,   0.8,   1.0 },
    { "M4",  4.1,   7.6,   0.9,   1.2 },
    { "M5",  5.1,   9.2,   1.2,   1.6 },
    { "M6",  6.1,  11.8,   1.6,   2.0 },
    { "M8",  8.1,  14.8,   2.0,   2.5 },
    { "M10",10.2,  18.1,   2.2,   3.0 },
    { "M12",12.2,  21.1,   2.5,   3.5 },
    { "M16",16.2,  27.4,   3.5,   4.5 },
    { "M20",20.2,  33.6,   4.0,   5.5 },
}};

inline const Din127Spec* findDin127(const std::string& size_key) {
    for (const auto& s : kDin127) if (size_key == s.size_key) return &s;
    return nullptr;
}

}  // namespace koocadcam::skill::retaining_rings
