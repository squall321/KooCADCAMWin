#pragma once
// @lat: [[engine/skills#linear_bushing_seat]]
//
// linear_bushing_seat — COMPOUND MACRO for an LM-bushing housing.
// Combines:
//   1. A bore sized for the bushing OD (slip fit, K7 typical).
//   2. A retention shoulder at the bottom (smaller diameter = inner shaft thru-hole).
//   3. Two lubrication grooves (radial slots cut into the bore wall).
//
//                  ┌─────────────────┐
//                  │    bushing OD   │   ← bushing seat (deep cyl pocket)
//                  │ ╔═════════════╗ │
//                  │ ║     ▓▓▓     ║ │   ← lub groove 1
//                  │ ║             ║ │
//                  │ ║     ▓▓▓     ║ │   ← lub groove 2
//                  │ ╚══╤═════════╤══╝ │
//                  │    │ shoulder│    │   ← retention shoulder
//                  │    │  thru   │    │
//                  └────┴─────────┴────┘
//
// Sub-feature count = 1 bore + 1 retention shoulder + 2 lub grooves = 4.
//
// Standards reference: ISO 10285 / DIN 1850 for LM-bushing housings,
// bushing OD typical 8–60 mm, retention shoulder = OD - 4 to 8 mm,
// lub-groove width = 1–3 mm, depth = 0.3–0.8 mm.
//
// DFM checks:
//   DFM-INPUT       : bushing_od, bushing_l > 0
//   DFM-002         : retention shoulder ID >= 0.8 mm
//   DFM-LB-GROOVE   : groove depth (0.5 mm) feasible (bushing_l >= 6 mm)
//   DFM-LB-SHOULDER : shoulder thickness >= 1 mm (OD-ID)/2 check
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace linear_bushing_seat {

constexpr const char* kSkillId = "linear_bushing_seat";

struct Input
{
    FaceDatum   entry_face;
    // Use a gp_Ax1 to fully define the seat axis (more flexible than entry_face + position).
    gp_Ax1      axis                { gp_Pnt(0,0,0), gp_Dir(0,0,-1) };
    double      bushing_od_mm       = 0.0;   // bushing outer diameter (the bore size)
    double      bushing_l_mm        = 0.0;   // bushing length (= bore depth)
    // Defaults:
    //   shoulder_offset_mm = 4.0 (shoulder ID = bushing_od - 2*4 = OD-8)
    //   groove_width_mm    = 1.5
    //   groove_depth_mm    = 0.5
    //   groove_positions   = bushing_l × {0.25, 0.75}
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace linear_bushing_seat
}  // namespace koocadcam::skill
