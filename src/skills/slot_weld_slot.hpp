#pragma once
// @lat: [[engine/skills#slot_weld_slot]]
//
// slot_weld_slot — COMPOUND MACRO for slot welding overlapping plates
// (elongated version of plug_weld_hole) per AWS A2.4.
//
// Geometric sub-features (≥ 3):
//   1. Center rectangular box cut (through the plate stack).
//   2. Two end-half-cylinder cuts (one at each end of the slot) — together
//      with the box they form a stadium-shape footprint.
//
//          ┌───────────────────────────┐
//          │  ╭─────────────────────╮  │
//          │  │ stadium-shape slot  │  │     slot_l  ↑
//          │  ╰─────────────────────╯  │     slot_w  →
//          └───────────────────────────┘
//
// AWS A2.4 slot guidance:
//   - slot_l ≥ 2 × slot_w (else use plug weld).
//   - slot_l ≤ 10 × plate_t (avoid distortion).
//   - corner_r = slot_w / 2 (end semicircles).
//
// DFM checks:
//   DFM-WELD-SLOT-LW     : slot_l ≥ 2 × slot_w.
//   DFM-WELD-SLOT-WIDTH  : slot_w ≥ 3 mm.
//
// Recognize: metadata replay only (confidence = 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace slot_weld_slot {

constexpr const char* kSkillId = "slot_weld_slot";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;        // slot center XY
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      slot_l        = 0.0;        // total length (along +X local)
    double      slot_w        = 0.0;        // width (along +Y local)
    gp_Dir      slot_dir      { 1.0, 0.0, 0.0 };  // direction the slot lies along
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace slot_weld_slot
}  // namespace koocadcam::skill
