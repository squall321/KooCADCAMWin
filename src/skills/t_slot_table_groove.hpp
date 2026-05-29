#pragma once
// @lat: [[engine/skills#t_slot_table_groove]]
//
// t_slot_table_groove — COMPOUND MECHANICAL FEATURE: a T-shaped fixture
// groove cut into a machine-table top.  The T-bolt head slides in the
// bottom (wider) channel; the bolt shank rides in the top (narrow) channel.
//
// This is the COMPOUND macro form that emphasises the T-cross-section as
// TWO STACKED RECTANGULAR CUTS (top slot then bottom slot) consistent with
// machinist practice: end-mill the top slot first to standard depth, then
// finish with a T-slot cutter for the wider bottom channel.
//
//   Side view (cut into top face):
//
//     ┌─ table top (entry_face)
//     ▼
//     ────────┬────────────────────────────────────────┬─────────
//             │  ┌──────────────────────────────────┐  │      ↑
//             │  │   top slot (narrow, top_w)       │  │      │ top_d
//             │  └──────────────────────────────────┘  │      ↓
//             │ ┌───────────────────────────────────┐  │      ↑
//             │ │                                   │  │      │
//             │ │  bottom slot (wide, bot_w)         │      │ bot_d
//             │ │                                   │  │      │
//             │ └───────────────────────────────────┘  │      ↓
//             └─────────────────────────────────────────────────
//
//   End view (looking along the slot):
//
//     ┌──┐                ← top slot (top_w wide, top_d deep, from top face)
//   ┌─┘  └─┐              ← bottom slot (bot_w wide, bot_d deep, BELOW top)
//   └──────┘
//
// Sub-features (2):
//   1. Top narrow rectangular slot — top_w × top_d.
//   2. Bottom wider rectangular slot — bot_w × bot_d.
// The two are fused into a single T-cross-section cutter and Boolean-cut
// in one operation (fuse_first_then_cut, same pattern as T_slot.cpp).
//
// Input:
//   - face_id   : entry face (table top)
//   - start/end : slot path endpoints (XY)
//   - top_w     : top slot width  (mm)
//   - top_d     : top slot depth  (mm, measured from face down)
//   - bot_w     : bottom slot width (mm, must be > top_w)
//   - bot_d     : bottom slot depth (mm, below the top slot)
//
// DFM (DIN 650 / ISO 299 table T-slot conventions):
//   DFM-002       top_w  < 0.8 mm
//   DFM-002       bot_w  < 0.8 mm
//   DFM-TSLOT     bot_w ≤ top_w (else it's a plain keyway)
//   DFM-DIN650    bot_w / top_w outside [1.5, 3.5] (DIN 650 typical ratio)
//   DFM-INPUT     top_d ≤ 0 or bot_d ≤ 0
//   DFM-INPUT     slot length ≤ 0
//
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace t_slot_table_groove {

constexpr const char* kSkillId = "t_slot_table_groove";

struct Input
{
    FaceDatum  entry_face;
    double     start_x_mm = 0.0;
    double     start_y_mm = 0.0;
    double     end_x_mm   = 0.0;
    double     end_y_mm   = 0.0;
    gp_Dir     axis_dir { 0.0, 0.0, -1.0 };

    double     top_w_mm = 0.0;
    double     top_d_mm = 0.0;
    double     bot_w_mm = 0.0;
    double     bot_d_mm = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace t_slot_table_groove
}  // namespace koocadcam::skill
