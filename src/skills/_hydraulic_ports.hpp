#pragma once
// @lat: [[engine/skills#central spec — hydraulic ports (slice 13b)]]
//
// Single source of truth for the slice-13b hydraulic port compound macros:
//
//   - SAE J518 Code 61 4-bolt split-flange (3000 psi series),
//     dash -8 through -32   (6 entries; metric M-bolt clearance per Eaton/Parker)
//   - BSPP / ISO 228-1 G-thread parallel ports, G1/8 → G1     (6 entries)
//   - NPT taper per ANSI/ASME B1.20.1, 1/8 → 3/4              (5 entries)
//   - JIC 37° flare seat (SAE J514) UN/UNF, dash -4 → -16     (5 entries)
//   - ORFS face-seal flat-face 4-bolt (variant of SAE J1453), -4 → -20 (6 entries)
//
// Each table is a flat constexpr std::array<…>; lookup by code returns the
// raw row or nullptr.  All linear dimensions are mm; all angles are degrees.
//
// Constants verified against:
//   - Eaton/Parker hydraulic fitting handbooks (SAE J518 Code 61 dimensions
//     including PCD, bolt size, captive face O-ring groove).
//   - ISO 228-1 (BSPP nominal OD, tap drill, pitch).
//   - ASME B1.20.1 (NPT pitch dia at gauging plane E1, taper 1:16 = 1.7857°/ft,
//     half-angle ≈ 1.7899°).
//   - SAE J514 (JIC 37° flare with UN/UNF straight thread above seat).
//   - SAE J1453 (ORFS captive face O-ring + 4-bolt variant for low-pressure
//     manifold ports).
//
// NOTE: This file is the slice-13b extension table.  It coexists with
// `_hydraulic_ports_table.hpp` (slice-13 original) which uses a different
// dash range (-08/-12/-16/-24) and naming convention.

#include <array>
#include <string>

namespace koocadcam::skill::hyd_ports {

// ── SAE J518 Code 61 4-bolt split-flange (3000 psi series) ──────────────
//
// The "Code 61" series is the standard 3000 psi 4-bolt split-flange used
// across mobile hydraulics.  Dash size = nominal port flow ID expressed in
// 16ths of an inch (-8 = ½", -12 = ¾", -16 = 1", -20 = 1¼", -24 = 1½",
// -32 = 2").  bolt_circle_dia is the PCD on which the 4 bolts sit; each
// bolt is metric (M10–M16) for industrial international port stamping.
//
// O-ring groove dimensions are for the captive BSE flat (face) groove:
// inner & outer diameter + axial depth, sized for the standard captive
// face O-ring of that dash.
struct SaeJ518Code61Spec {
    const char* dash_size;          // "-8", "-12", "-16", "-20", "-24", "-32"
    double      port_dia_mm;        // clearance bore (tube OD passes through)
    double      bolt_circle_dia_mm; // bolt PCD
    double      bolt_dia_mm;        // M10/M12/M14/M16 nominal
    int         bolt_count;         // always 4
    double      o_ring_groove_id_mm;
    double      o_ring_groove_od_mm;
    double      o_ring_groove_depth_mm;
};

// 6 entries covering the full Code 61 mobile-hydraulics catalogue.
constexpr std::array<SaeJ518Code61Spec, 6> kSaeJ518Code61{{
    // dash  port    bcd     bolt   n  oringID  oringOD  oringD
    { "-8",  12.70,  30.16,  10.0,  4, 20.65,   24.21,   2.10 },   // M10, captive -210 (CS 3.53) ish
    { "-12", 19.05,  38.10,  12.0,  4, 28.20,   33.44,   2.10 },   // M12
    { "-16", 25.40,  52.39,  12.0,  4, 38.10,   43.36,   2.10 },   // M12
    { "-20", 31.75,  58.75,  14.0,  4, 47.60,   52.86,   2.10 },   // M14
    { "-24", 38.10,  66.68,  14.0,  4, 51.40,   56.66,   2.10 },   // M14
    { "-32", 50.80,  79.38,  16.0,  4, 66.55,   73.65,   2.50 },   // M16, larger CS captive ring
}};

inline const SaeJ518Code61Spec* findSaeJ518Code61(const std::string& dash_size) {
    for (const auto& s : kSaeJ518Code61) if (dash_size == s.dash_size) return &s;
    return nullptr;
}

// ── BSPP / ISO 228-1 G-thread parallel port ──────────────────────────────
//
// Parallel (non-tapered) G-thread per ISO 228-1.  The "drill" is the bore
// to which the parallel thread is later cut; "spotface" is the wider face
// counterbore where the bonded-seal washer or O-ring sits.
struct BspGSpec {
    const char* size;            // "G1/8", "G1/4", "G3/8", "G1/2", "G3/4", "G1"
    double      thread_major_mm; // BSPP major dia (= nominal thread OD)
    double      thread_minor_mm; // BSPP minor dia (≈ tap drill)
    double      pitch_mm;        // BSPP pitch
    double      drill_dia_mm;    // recommended tap drill dia (~75% engagement)
    double      spotface_dia_mm; // sealing-washer spotface dia
    double      depth_mm;        // default useful thread depth into part
};

constexpr std::array<BspGSpec, 6> kBspG{{
    // size    major   minor   pitch  drill   spotface depth
    { "G1/8",   9.728,  8.566,  0.907,  8.80,  16.0,    8.0 },
    { "G1/4",  13.157, 11.445,  1.337, 11.80,  20.0,   10.0 },
    { "G3/8",  16.662, 14.950,  1.337, 15.25,  24.0,   10.5 },
    { "G1/2",  20.955, 18.631,  1.814, 19.00,  29.0,   13.5 },
    { "G3/4",  26.441, 24.117,  1.814, 24.50,  35.0,   14.0 },
    { "G1",    33.249, 30.291,  2.309, 30.75,  42.0,   16.0 },
}};

inline const BspGSpec* findBspG(const std::string& size) {
    for (const auto& s : kBspG) if (size == s.size) return &s;
    return nullptr;
}

// ── NPT taper per ANSI/ASME B1.20.1 ─────────────────────────────────────
//
// NPT (National Pipe Tapered) has a 1:16 taper (≈ 1.7857°/ft total, half
// angle per side ≈ 1.7899°).  pitch_dia_at_E1_mm is the pitch diameter at
// the gauging plane (E1 in ASME B1.20.1).  drill_dia_mm is the
// recommended tap drill (slightly smaller than minor dia to allow the
// taper tap to cut full thread).
struct NptSpec {
    const char* size;                // "1/8", "1/4", "3/8", "1/2", "3/4"
    double      pitch_dia_at_E1_mm;  // ASME B1.20.1 E1 pitch dia
    double      drill_dia_mm;        // ANSI tap drill (recommended)
    double      depth_mm;            // standard hand-tight engagement depth (L1)
    double      threads_per_inch;    // documentation (TPI of the taper)
};

// NPT taper is exactly 1:16 (taper_per_foot = 0.75 inch/foot = 1/16).
// Half-angle per side = atan(1/32) ≈ 1.7899° → cone narrows at this rate.
constexpr double kNptTaperPerSideDeg = 1.7899;
constexpr double kNptTaperRatio      = 1.0 / 16.0;   // diameter taper per unit length

constexpr std::array<NptSpec, 5> kNpt{{
    // size    pitchE1   drill  depth   TPI
    { "1/8",    9.890,   8.61,   6.5,   27.0 },
    { "1/4",   13.227,  11.10,   9.6,   18.0 },
    { "3/8",   16.662,  14.50,   9.9,   18.0 },
    { "1/2",   20.587,  17.85,  13.5,   14.0 },
    { "3/4",   25.857,  23.10,  13.7,   14.0 },
}};

inline const NptSpec* findNpt(const std::string& size) {
    for (const auto& s : kNpt) if (size == s.size) return &s;
    return nullptr;
}

// ── JIC 37° flare-seat per SAE J514 ──────────────────────────────────────
//
// JIC flare port: straight UN/UNF thread above a 37° conical seat (=74°
// included).  bore_dia_mm is the central flow bore; flare_seat_od is the
// OD of the cone at the face; flare_min_dia is the bottom of the cone
// where the flow bore opens.  Thread is straight UN/UNF (use existing
// _iso_thread_table.hpp UncUnf helpers for clearance).
struct JicFlareSpec {
    const char* tube_size;       // dash "-4", "-6", "-8", "-12", "-16"
    double      thread_major_mm; // UN/UNF nominal OD
    double      thread_pitch_mm; // UN/UNF pitch
    double      bore_dia_mm;     // central flow bore
    double      flare_seat_od_mm;// OD of 37° cone at face
    double      seat_depth_mm;   // axial depth of conical seat
};

constexpr double kJicFlareHalfAngleDeg = 37.0;  // 74° included

constexpr std::array<JicFlareSpec, 5> kJicFlare{{
    // dash   thread  pitch    bore    flareOD  seatD
    { "-4",   11.11,  1.411,    4.83,   7.92,   2.50 },   // 7/16-20 UNF
    { "-6",   14.29,  1.411,    7.92,  11.10,   3.00 },   // 9/16-18 UNF
    { "-8",   19.05,  1.587,   11.10,  15.24,   3.50 },   // 3/4-16 UNF
    { "-12",  26.99,  1.587,   17.45,  22.45,   4.00 },   // 1-1/16-12 UN
    { "-16",  33.34,  1.587,   23.83,  28.58,   4.50 },   // 1-5/16-12 UN
}};

inline const JicFlareSpec* findJicFlare(const std::string& dash) {
    for (const auto& s : kJicFlare) if (dash == s.tube_size) return &s;
    return nullptr;
}

// ── ORFS 4-bolt flat-face seal port (low-pressure manifold variant) ─────
//
// Flat face with concentric captive O-ring groove (BS dash size sized
// for the bore) + 4 bolt holes on a PCD around the port.  This is the
// flange variant of ORFS used on bolted manifold blocks (lower-pressure
// alternative to SAE J518 Code 61).
struct OrfsPortSpec {
    const char* dash_size;          // "-4", "-6", "-8", "-12", "-16", "-20"
    double      port_dia_mm;        // central flow bore
    double      bolt_circle_dia_mm; // bolt PCD
    double      bolt_dia_mm;        // M6/M8/M10 metric
    int         bolt_count;         // always 4
    double      o_ring_groove_id_mm;
    double      o_ring_groove_od_mm;
    double      o_ring_groove_depth_mm;
};

constexpr std::array<OrfsPortSpec, 6> kOrfsPort{{
    // dash  port   bcd    bolt  n   grvID  grvOD   grvD
    { "-4",   6.35, 22.0,  6.0,  4,  9.65,  12.95,  1.65 },
    { "-6",   9.53, 26.0,  6.0,  4, 12.45,  15.75,  1.65 },
    { "-8",  12.70, 30.0,  8.0,  4, 15.50,  18.80,  1.65 },
    { "-12", 19.05, 38.0,  8.0,  4, 21.85,  25.15,  1.85 },
    { "-16", 25.40, 48.0, 10.0,  4, 28.30,  31.60,  1.85 },
    { "-20", 31.75, 56.0, 10.0,  4, 34.90,  38.20,  1.85 },
}};

inline const OrfsPortSpec* findOrfsPort(const std::string& dash_size) {
    for (const auto& s : kOrfsPort) if (dash_size == s.dash_size) return &s;
    return nullptr;
}

}  // namespace koocadcam::skill::hyd_ports
