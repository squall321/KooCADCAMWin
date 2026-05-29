#pragma once
// @lat: [[engine/skills#sprocket_with_chain_teeth]]
//
// sprocket_with_chain_teeth — REAL compound feature:
//
//   sub-cuts (N + 1):
//     1. central through bore
//     2..N+1. N seating-pocket cutouts sized to chain roller diameter,
//             equispaced around the pitch circle.  Pocket = roller_dia
//             cylindrical cutter centered on each tooth pitch point.
//
// Chain standards (ANSI B29.1 — selected sizes):
//   ANSI #25: pitch 6.35 mm,  roller_dia 3.30 mm
//   ANSI #35: pitch 9.525 mm, roller_dia 5.08 mm
//   ANSI #40: pitch 12.7 mm,  roller_dia 7.92 mm
//   ANSI #50: pitch 15.875 mm, roller_dia 10.16 mm
//   ANSI #60: pitch 19.05 mm, roller_dia 11.91 mm
//   ANSI #80: pitch 25.4 mm,  roller_dia 15.88 mm
//
// Pitch diameter (PD) for a sprocket with N teeth on chain pitch P:
//   PD = P / sin(180° / N)
//
// DFM gates per ANSI B29.1:
//   DFM-CHAIN-SIZE  unknown chain size
//   DFM-CHAIN-N     N < 9 (chain wraps inadequately) or > 120
//   DFM-CHAIN-OD    OD = PD + roller_dia must fit blank
//   DFM-CHAIN-BORE  bore_dia collides with PD

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sprocket_with_chain_teeth {

constexpr const char* kSkillId = "sprocket_with_chain_teeth";

struct Input
{
    double      blank_dia_mm     = 0.0;   // raw OD
    double      blank_thick_mm   = 0.0;
    std::string ansi_chain_size  = "";    // "25" | "35" | "40" | "50" | "60" | "80"
    int         num_teeth        = 0;     // 9..120
    double      bore_dia_mm      = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Helpers for chain lookup.
struct ChainSize { double pitch_mm; double roller_dia_mm; };
bool lookupChain(const std::string& ansi, ChainSize& out);

}  // namespace sprocket_with_chain_teeth
}  // namespace koocadcam::skill
