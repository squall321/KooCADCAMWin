#pragma once
// @lat: [[engine/skills#plug_weld_hole]]
//
// plug_weld_hole — COMPOUND MACRO for plug welding overlapping plates per
// AWS A2.4 / ANSI/AWS A3.0.
//
// Geometric sub-features (≥ 2):
//   1. Through cylindrical hole — full thickness, hole_dia 3 mm — 12 mm.
//   2. Countersink on the welder-side face (one side only) — 45° included
//      angle so the puddle wets the top face cleanly.
//
//                        ┌──────────────────┐  ← entry face
//                        │\                /│  ← 45° countersink
//                        │ \              / │
//                        │  ┌─┐        ┌─┐  │
//                        │  │ │        │ │  │  ↑
//                        │  │ │ plug   │ │  │  │ plate stack (sum of plate
//                        │  │ │ weld   │ │  │  │ thicknesses; through-hole
//                        │  │ │ hole   │ │  │  │ pierces the entire stack)
//                        │  │ │        │ │  │  ↓
//                        └──┴─┴────────┴─┴──┘
//                                ↕
//                              hole_dia
//
// AWS A2.4 hole size: 3 mm ≤ hole_dia ≤ 12 mm typically.  Countersink
// included angle 45° (90° / 2).
//
// DFM checks (AWS A2.4):
//   DFM-WELD-PLUG-DIA   : hole_dia in [3.0, 12.0].
//   DFM-002             : hole_dia ≥ 0.8 mm (universal drill min).
//
// Recognize: metadata replay only (confidence = 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace plug_weld_hole {

constexpr const char* kSkillId = "plug_weld_hole";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      hole_dia      = 6.0;          // 3 mm — 12 mm per AWS
    double      cs_top_dia    = 0.0;          // 0 → defaults to hole_dia + 2 mm
    double      cs_angle_deg  = 90.0;         // included angle (45° per side)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace plug_weld_hole
}  // namespace koocadcam::skill
