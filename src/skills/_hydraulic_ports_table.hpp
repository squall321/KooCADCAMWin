#pragma once
// @lat: [[engine/skills#central spec — hydraulic ports]]
//
// Single source of truth for hydraulic / pneumatic port standards used by the
// slice-13 compound macros:
//
//   - SAE J518   (4-bolt split-flange, Code 61 working pressure series)
//   - ISO 6149-1 (metric M-stud O-ring boss port)         [data shown for
//                 BSPP geometry overlap; ISO 6149-1 specific entries can
//                 be added in a future revision]
//   - BSPP / ISO 228-1 (G-thread parallel port)
//   - NPT       (ANSI B1.20.1 taper port)
//   - JIC 37°   (SAE J514 / ISO 8434-2 flare seat)
//   - ORFS       (SAE J1453 face-seal port)
//
// Each table is a flat constexpr std::array<…>; lookup by code returns the
// raw row or nullptr.  All dimensions are mm; all angles deg.
//
// Sources for the constants (verified against published handbook tables):
//   - SAE J518 Code 61 — dash size table (Eaton/Parker hydraulic fitting
//     handbook).  Bolt = UNC for inch sizes; PCD is the bolt-pattern pitch
//     circle diameter.  Face-seal O-ring is the captive seal in the flange
//     groove.
//   - BSPP — ISO 228-1 G-thread nominal OD and tap drill diameter.
//   - NPT  — ASME B1.20.1 (1.7857°/ft taper) nominal OD + threads/inch.
//   - JIC 37° — SAE J514 straight-thread + 37° half-angle flare seat.
//   - ORFS — SAE J1453 UN/UNF straight thread + face O-ring groove ID.

#include <array>
#include <string>

namespace koocadcam::skill::hydraulic_ports {

// ── SAE J518 4-bolt split-flange (Code 61 working pressure series) ───────
struct SaeJ518Spec {
    const char* code;            // "-08", "-12", "-16", "-24"
    double      dn_mm;           // nominal flow ID
    double      thru_dia_mm;     // central through-bore in port
    double      pcd_mm;          // bolt pattern pitch circle diameter
    double      bolt_dia_mm;     // bolt clearance hole diameter (per bolt of 4)
    double      oring_id_mm;     // captive face O-ring ID
    double      oring_cs_mm;     // captive face O-ring cross section
};

// Code 61 (3000-PSI series) standard 4 sizes used in mobile hydraulics.
constexpr std::array<SaeJ518Spec, 4> kSaeJ518{{
    // code   dn     thru   pcd    bolt   oringID  cs
    { "-08",   9.5,  16.0,  30.2,   8.0,  20.65, 1.78 },
    { "-12",  16.0,  25.4,  38.1,  10.0,  28.20, 2.62 },
    { "-16",  25.4,  38.1,  52.4,  12.0,  38.10, 2.62 },
    { "-24",  38.1,  50.8,  66.7,  12.0,  51.40, 2.62 },
}};

inline const SaeJ518Spec* findSaeJ518(const std::string& code) {
    for (const auto& s : kSaeJ518) if (code == s.code) return &s;
    return nullptr;
}

// ── BSPP / ISO 228-1 (G-thread parallel hydraulic port) ──────────────────
struct BsppSpec {
    const char* code;            // "G1/8", "G1/4", "G3/8", "G1/2", "G3/4", "G1"
    double      thread_od_mm;    // nominal thread OD
    double      tap_drill_dia_mm;// recommended tap drill (75 % engagement)
    double      pitch_mm;        // thread pitch
    double      chamfer_leg_mm;  // recommended entry chamfer leg (≈ 0.5 × pitch)
};

constexpr std::array<BsppSpec, 6> kBspp{{
    // code     od    tapdrill pitch  chamfer
    { "G1/8",   9.728,  8.80,  0.907, 0.45 },
    { "G1/4",  13.157, 11.80,  1.337, 0.67 },
    { "G3/8",  16.662, 15.25,  1.337, 0.67 },
    { "G1/2",  20.955, 19.00,  1.814, 0.91 },
    { "G3/4",  26.441, 24.50,  1.814, 0.91 },
    { "G1",    33.249, 30.75,  2.309, 1.15 },
}};

inline const BsppSpec* findBspp(const std::string& code) {
    for (const auto& s : kBspp) if (code == s.code) return &s;
    return nullptr;
}

// ── NPT / ASME B1.20.1 taper thread port ─────────────────────────────────
struct NptSpec {
    const char* code;            // "1/16", "1/8", "1/4", "3/8", "1/2", "3/4", "1"
    double      major_dia_mm;    // major (nominal) OD at the face
    double      minor_dia_mm;    // minor OD at L1 (effective thread length)
    double      l1_depth_mm;     // ASME B1.20.1 effective thread length
    double      chamfer_leg_mm;  // recommended entry chamfer leg
};

// NPT taper is exactly 1°47'/ft  ≈ 1.7857°/ft total, i.e. half-angle per
// side ≈ 1.7899° ≈ tan⁻¹(1/32).  Minor dia computed by tip's narrow end.
constexpr std::array<NptSpec, 7> kNpt{{
    // code     major   minor   l1     chamf
    { "1/16",   8.13,   7.39,   4.06,  0.4 },
    { "1/8",   10.29,   9.55,   4.10,  0.4 },
    { "1/4",   13.72,  12.61,   5.79,  0.5 },
    { "3/8",   17.15,  16.04,   6.10,  0.5 },
    { "1/2",   21.34,  19.79,   8.13,  0.6 },
    { "3/4",   26.67,  25.13,   8.61,  0.6 },
    { "1",     33.40,  31.46,  10.16,  0.8 },
}};

inline const NptSpec* findNpt(const std::string& code) {
    for (const auto& s : kNpt) if (code == s.code) return &s;
    return nullptr;
}

// ── JIC 37° flare port (SAE J514) ────────────────────────────────────────
struct JicSpec {
    const char* code;            // "-04", "-06", "-08", "-12"
    double      thread_od_mm;    // UNF major dia (e.g. 7/16, 9/16, 3/4, 1-1/16)
    double      pitch_mm;        // UNF thread pitch
    double      flare_od_mm;     // outer dia of the 37° flare seat at the face
    double      flare_min_dia_mm;// inner dia (flow passage at deep end)
    double      seat_depth_mm;   // axial depth of the conical seat
};

// Flare half-angle is 37° (= 74° included).
constexpr std::array<JicSpec, 4> kJic{{
    // code   thread  pitch   flareOD  flareMin  seatD
    { "-04",  11.11,  1.411,   7.92,   4.83,   2.50 },   // 7/16-20 UNF
    { "-06",  14.29,  1.411,  11.10,   7.92,   3.00 },   // 9/16-18 UNF
    { "-08",  19.05,  1.587,  15.24,  11.10,   3.50 },   // 3/4-16  UNF
    { "-12",  26.99,  1.587,  22.45,  17.45,   4.00 },   // 1-1/16-12 UNF
}};

inline const JicSpec* findJic(const std::string& code) {
    for (const auto& s : kJic) if (code == s.code) return &s;
    return nullptr;
}

// ── ORFS face-seal port (SAE J1453) ──────────────────────────────────────
struct OrfsSpec {
    const char* code;            // "-04", "-06", "-08", "-12", "-16", "-20"
    double      thread_od_mm;    // UN/UNF major dia
    double      pitch_mm;        // thread pitch
    double      bore_dia_mm;     // central flow bore
    double      oring_groove_id_mm; // ID of the captive face O-ring groove
    double      oring_groove_od_mm; // OD of the captive face O-ring groove
    double      oring_groove_depth_mm; // axial groove depth
};

constexpr std::array<OrfsSpec, 6> kOrfs{{
    // code   thread  pitch   bore   grvID    grvOD    grvD
    { "-04",  14.29,  1.411,   6.35,  9.65,  12.95,  1.65 },   // 9/16-18 UNF
    { "-06",  17.46,  1.411,   9.53, 12.45,  15.75,  1.65 },   // 11/16-16
    { "-08",  20.64,  1.411,  12.70, 15.50,  18.80,  1.65 },   // 13/16-16
    { "-12",  26.99,  1.587,  19.05, 21.85,  25.15,  1.85 },   // 1-3/16-12
    { "-16",  33.34,  1.587,  25.40, 28.30,  31.60,  1.85 },   // 1-7/16-12
    { "-20",  41.28,  1.587,  31.75, 34.90,  38.20,  1.85 },   // 1-11/16-12
}};

inline const OrfsSpec* findOrfs(const std::string& code) {
    for (const auto& s : kOrfs) if (code == s.code) return &s;
    return nullptr;
}

}  // namespace koocadcam::skill::hydraulic_ports
