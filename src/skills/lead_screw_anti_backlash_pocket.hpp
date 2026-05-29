#pragma once
// @lat: [[engine/skills#lead_screw_anti_backlash_pocket]]
//
// lead_screw_anti_backlash_pocket — COMPOUND MACRO for an anti-backlash
// half-nut assembly seat.  Combines:
//   1. Two stacked half-nut pockets (cylinders, separated by spring slot).
//   2. An axial spring-relief slot (thin rectangular gap).
//   3. An adjustment screw pilot (small thru-hole, transverse to nut axis).
//
//              ┌──────────────────────┐
//              │  ╭───────────────╮   │   ← upper half-nut pocket
//              │  │   half-nut    │   │
//              │  ╰───────────────╯   │
//              │  ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓   │   ← spring slot (thin)
//              │  ╭───────────────╮   │
//              │  │   half-nut    │   │   ← lower half-nut pocket
//              │  ╰───────────────╯   │
//              │      ● adj-screw     │   ← pilot hole (M3, transverse)
//              └──────────────────────┘
//
// Sub-feature count = 2 half-nut pockets + 1 spring slot + 1 adj-screw pilot = 4.
//
// Standards reference: classical anti-backlash lead-screw nut (Roton, Nook),
// spring slot width typ. 0.5–2 mm, adj-screw is M3 perpendicular.
//
// DFM checks:
//   DFM-INPUT       : half_nut_od, half_nut_l, spring_slot_w > 0
//   DFM-002         : derived adj-screw pilot (2.5 mm for M3) >= 0.8 mm
//   DFM-AB-SLOT     : spring_slot_w in [0.3, 5.0] mm
//   DFM-AB-WALL     : wall between two half-nut pockets >= 1 mm
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lead_screw_anti_backlash_pocket {

constexpr const char* kSkillId = "lead_screw_anti_backlash_pocket";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm       = 0.0;   // center of nut stack
    double      position_y_mm       = 0.0;
    gp_Dir      axis_dir            { 0.0, 0.0, -1.0 };
    double      half_nut_od_mm      = 0.0;   // each half-nut pocket diameter
    double      half_nut_l_mm       = 0.0;   // each half-nut pocket depth
    double      spring_slot_w_mm    = 0.0;   // thin gap between the two pockets
    // Defaults:
    //   adj_screw_pilot_dia_mm = 2.5 (M3 pilot)
    //   adj_screw_depth_mm     = half_nut_od_mm × 1.2
    //   spring_slot_extra_mm   = 2.0 (slot extends 2 mm past pocket on each side)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lead_screw_anti_backlash_pocket
}  // namespace koocadcam::skill
