#pragma once
#include <array>
#include <string>

namespace koocadcam::skill::thread_table {

// Single row covers: thread spec, pilot/tap, clearance fits, head sizes.
struct MetricThreadSpec {
    const char* size_key;          // "M3", "M4", "M5", "M6", "M8", "M10", "M12", "M16", "M20", "M24", "M30"
    double nominal_dia_mm;         // exact thread OD (3.0, 4.0, etc.)
    double pitch_mm;               // coarse pitch (0.5, 0.7, 0.8, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 3.5)
    double tap_pilot_dia_mm;       // ISO 965 60% engagement pilot drill
    double clearance_close_mm;     // ISO 273 close fit clearance
    double clearance_medium_mm;    // ISO 273 medium fit clearance (most common)
    double clearance_free_mm;      // ISO 273 free fit clearance
    double socket_head_dia_mm;     // ASME B18.3 socket head OD
    double socket_head_height_mm;  // ASME B18.3 socket head height
    double countersink_82_dia_mm;  // 82-degree CSK reference diameter for inch screws (mostly empty for M)
    double countersink_90_dia_mm;  // 90-degree CSK reference diameter (ISO 7721)
};

// 11 standard entries covering the most common metric M sizes.
constexpr std::array<MetricThreadSpec, 11> kMetricThreads {{
    // size      nom  pit  pilot  close med   free  SHd   SHh   CSK82 CSK90
    { "M3",      3.0, 0.5, 2.5,   3.2,  3.4,  3.6,  5.5,  3.0,  0.0,  6.3 },
    { "M4",      4.0, 0.7, 3.3,   4.3,  4.5,  4.8,  7.0,  4.0,  0.0,  8.4 },
    { "M5",      5.0, 0.8, 4.2,   5.3,  5.5,  5.8,  8.5,  5.0,  0.0, 10.4 },
    { "M6",      6.0, 1.0, 5.0,   6.4,  6.6,  7.0, 10.0,  6.0,  0.0, 12.6 },
    { "M8",      8.0, 1.25, 6.8,  8.4,  9.0, 10.0, 13.0,  8.0,  0.0, 17.3 },
    { "M10",    10.0, 1.5, 8.5,  10.5, 11.0, 12.0, 16.0, 10.0,  0.0, 20.0 },
    { "M12",    12.0, 1.75, 10.2, 13.0, 14.0, 15.0, 18.0, 12.0,  0.0, 24.0 },
    { "M16",    16.0, 2.0, 14.0, 17.0, 18.0, 19.0, 24.0, 16.0,  0.0, 30.0 },
    { "M20",    20.0, 2.5, 17.5, 21.0, 22.0, 24.0, 30.0, 20.0,  0.0, 35.0 },
    { "M24",    24.0, 3.0, 21.0, 25.0, 26.0, 28.0, 36.0, 24.0,  0.0, 40.0 },
    { "M30",    30.0, 3.5, 26.5, 31.0, 33.0, 35.0, 45.0, 30.0,  0.0, 50.0 },
}};

// Lookup by size_key ("M6"). Returns nullptr if not in table.
inline const MetricThreadSpec* findMetric(const std::string& size_key) {
    for (const auto& s : kMetricThreads) if (size_key == s.size_key) return &s;
    return nullptr;
}

// Same shape for UNC/UNF imperial (separate table).
// Clearance values follow ASME B18.2.8 (normal/close/loose fit classes),
// the standard reference for imperial machine-screw clearance holes.
struct UncUnfSpec {
    const char* size_key;          // "#2-56", "#4-40", "#6-32", "#8-32", "#10-24", "1/4-20", "3/8-16", "1/2-13"
    double nominal_dia_mm;
    double tap_pilot_dia_mm;
    double clearance_normal_mm;    // ASME B18.2.8 normal fit
    double clearance_close_mm;     // ASME B18.2.8 close fit
    double clearance_loose_mm;     // ASME B18.2.8 loose fit
};

constexpr std::array<UncUnfSpec, 8> kUncUnfThreads {{
    // size       nom    pilot  normal close loose      (ASME B18.2.8, mm)
    { "#2-56",    2.18,  1.85,  2.39,  2.69,  2.95 },
    { "#4-40",    2.84,  2.40,  2.95,  3.56,  3.78 },
    { "#6-32",    3.51,  3.00,  3.66,  4.22,  4.50 },
    { "#8-32",    4.17,  3.55,  4.32,  4.98,  5.23 },
    { "#10-24",   4.83,  4.10,  4.98,  5.61,  5.79 },
    { "1/4-20",   6.35,  5.40,  7.14,  7.54,  8.20 },
    { "3/8-16",   9.53,  8.20, 10.31, 10.72, 11.51 },
    { "1/2-13",  12.70, 10.90, 13.49, 14.27, 15.47 },
}};

inline const UncUnfSpec* findUncUnf(const std::string& size_key) {
    for (const auto& s : kUncUnfThreads) if (size_key == s.size_key) return &s;
    return nullptr;
}

}  // namespace
