#pragma once
// @lat: [[engine/skills#over_center_toggle_pocket]]
//
// over_center_toggle_pocket — pocket for over-center toggle clamp.
//
// Over-center toggles are bi-stable mechanisms that snap between two stable
// positions, used in latches, draw clamps, and quick-release fittings
// (ISO 3402, DIN 6840).  The pocket needs:
//   - PIVOT BORE (the toggle pin axis)
//   - CAM SLOT (arc-shaped path for the cam follower)
//   - OVER-CENTER STOP (small ridge or notch that gives the "snap")
//   - SPRING ANCHOR HOLE (locates the bias spring)
//
// COMPOUND CHAIN (≥ 4 primitive cuts/fuses):
//   1. PIVOT BORE cut
//   2. CAM SLOT cut (rectangular slot offset from pivot)
//   3. OVER-CENTER STOP cut (small notch at the slot midpoint)
//   4. SPRING ANCHOR HOLE (small bore offset from pivot)
//
// Spec table (toggle_class → DIN 6840 toggle clamps):
//   "TC-A20" : pivot_dia 3, slot_l 10, slot_w 2, anchor_dia 2,  stop_w 1
//   "TC-A40" : pivot_dia 5, slot_l 18, slot_w 3, anchor_dia 3,  stop_w 1.5
//   "TC-A80" : pivot_dia 8, slot_l 30, slot_w 5, anchor_dia 5,  stop_w 2.5
//
// DFM gates:
//   DFM-002    : all widths ≥ 0.8 mm
//   DFM-PIVOT  : pivot bore depth ≥ 1.5×pivot_dia
//   DFM-SPEC   : toggle_class must be in spec table

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace over_center_toggle_pocket {

constexpr const char* kSkillId = "over_center_toggle_pocket";

struct Input
{
    FaceDatum    entry_face;
    double       pivot_x_mm    = 0.0;
    double       pivot_y_mm    = 0.0;
    gp_Dir       axis_dir      { 0.0, 0.0, -1.0 };
    double       pocket_depth_mm = 0.0;       // base depth for all sub-cuts
    double       slot_angle_deg  = 0.0;       // rotation of cam slot about pivot
    std::string  toggle_class    = "TC-A40";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

struct ToggleSpec {
    const char* key;
    double      pivot_dia_mm;
    double      slot_length_mm;
    double      slot_width_mm;
    double      anchor_dia_mm;
    double      stop_width_mm;
};
const ToggleSpec* lookupToggleSpec(const std::string& key);

}  // namespace over_center_toggle_pocket
}  // namespace koocadcam::skill
