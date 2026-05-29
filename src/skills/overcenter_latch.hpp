#pragma once
// @lat: [[engine/skills#overcenter_latch]]
//
// overcenter_latch — toggle / over-center latch body cut into a plate.
//
// REAL geometry chain (4 cuts):
//   1. Lever-arm pocket — rectangular slot for the lever to swing in.
//   2. Cam slot — curved-end slot that lets the cam pin travel along the
//      over-center arc.  Modelled in slice-1 as a rectangular slot with
//      semicircular ends (stadium / rounded-rect).
//   3. Pivot bore — cylindrical hole at one end of the lever pocket for the
//      lever pivot pin.
//   4. Retention hook pocket — a small box at the FAR end of the lever
//      pocket that retains the keeper.
//   5. Handle grip notch — chamfer slot on the entry face for finger grip.
//
// Standard:
//   - Agent-chosen (DIN 3017 hose-clamp lever and Southco E5 toggle-latch
//     style geometry — no single ISO standard covers over-center latches).
//   - DFM-002 (min pocket width 0.8 mm) and DFM-004 (corner R ≥ 0.2 mm)
//     applied conservatively.
//
// DFM gates:
//   - lever_arm_length_mm ≥ 6 mm (functional minimum).
//   - pivot_bore_dia_mm   ≥ 1.6 mm (M1.6 minimum pin) AND
//     pivot_bore_dia_mm < cam_slot_width_mm × 4 (sane proportion).
//   - cam_slot_length_mm > cam_slot_width_mm (else not a slot).
//   - retention_hook_depth_mm ≤ pivot_bore_depth_mm (else hook breaks through).
//
// Signature:
//   - is_compound = true
//   - subfeature_count = 5
//
// Recognize:
//   - PRIMARY (geometric): look for a single pivot cylinder AND a long
//     rectangular pocket with non-trivial aspect ratio (lever-arm pocket).
//   - FALLBACK (metadata replay): confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace overcenter_latch {

constexpr const char* kSkillId = "overcenter_latch";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm        = 0.0;       // lever pivot XY
    double      position_y_mm        = 0.0;
    gp_Dir      axis_dir             { 0.0, 0.0, -1.0 };
    double      lever_arm_length_mm  = 30.0;      // along +X from pivot
    double      lever_arm_width_mm   = 6.0;       // along Y
    double      lever_pocket_depth_mm = 4.0;
    double      pivot_bore_dia_mm    = 3.0;
    double      pivot_bore_depth_mm  = 6.0;
    double      cam_slot_offset_mm   = 8.0;       // distance from pivot to cam slot start
    double      cam_slot_length_mm   = 6.0;
    double      cam_slot_width_mm    = 2.5;
    double      cam_slot_depth_mm    = 3.0;
    double      retention_hook_length_mm = 4.0;
    double      retention_hook_width_mm  = 3.0;
    double      retention_hook_depth_mm  = 2.5;
    double      grip_notch_width_mm  = 2.0;
    double      grip_notch_depth_mm  = 1.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace overcenter_latch
}  // namespace koocadcam::skill
