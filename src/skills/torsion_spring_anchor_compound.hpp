#pragma once
// @lat: [[engine/skills#torsion_spring_anchor_compound]]
//
// torsion_spring_anchor_compound — COMPOUND FEATURE (4 primitive sub-cuts).
//
// Realistic torsion-spring mounting on a metal frame.  A central pivot
// bore receives the spring coil (axial), an arm-clearance slot allows the
// long arm of the torsion spring to swing through ±θ, and 2 anchor-leg
// pilot holes capture the short anchor leg of the spring against the
// frame.
//
//   Sub-feature chain:
//     1. Pivot bore        (cylindrical bore for the coil OD slip fit)
//     2. Arm-clearance slot (radial slot extending from pivot bore in
//                            arm direction — width = arm_slot.width,
//                            length = arm_slot.length, depth = pivot_d)
//     3. Anchor-leg pilot 1 (drill pilot for the spring's anchor leg
//                            at angle θ₁ from arm axis)
//     4. Anchor-leg pilot 2 (drill pilot for the spring's anchor leg
//                            at angle θ₂ from arm axis)
//
//   All four cutters are compounded then a single Boolean cut() removes
//   them together.  Pivot bore and arm slot are concentric on the entry
//   face plane (slot extends radially out from the bore wall).
//
// Output topology: 1 pivot cylinder + 2 slot walls + 1 slot bottom +
// 2 small cylindrical pilot bores.  Single Workpiece, single
// FeatureSignature with pattern.is_compound = true, subfeature_count = 4.
//
// DFM gates (ISO 8459 torsion-spring assembly + DIN 2089-2):
//   - pivot_bore >= 3 mm (covered torsion-spring coil ID range)
//   - arm_slot.width >= pivot_bore × 0.15 (spring wire dia ≥ 15% of pivot)
//   - arm_slot.length <= pivot_bore × 3 (avoid excessive material loss)
//   - anchor pilot_dia = 2 × spring wire estimate = arm_slot.width × 0.8
//     must be ≥ 0.8 mm
//   - frame thickness ≥ pivot bore depth + 2 mm wall

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace torsion_spring_anchor_compound {

constexpr const char* kSkillId = "torsion_spring_anchor_compound";

struct Input
{
    FaceDatum   face_id;
    double      position_x_mm    = 0.0;
    double      position_y_mm    = 0.0;
    gp_Dir      axis_dir         { 0.0, 0.0, -1.0 };
    double      pivot_bore       = 0.0;       // bore diameter, mm
    double      pivot_depth_mm   = 0.0;       // bore depth, mm
    double      arm_slot_width   = 0.0;       // arm-clearance slot width, mm
    double      arm_slot_length  = 0.0;       // arm-clearance slot radial length, mm
    double      anchor_angle_1_deg = 30.0;    // angle to anchor pilot 1 (from +X)
    double      anchor_angle_2_deg = -30.0;   // angle to anchor pilot 2 (from +X)
    double      anchor_radius_mm  = 0.0;      // distance from pivot center to anchor pilots
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace torsion_spring_anchor_compound
}  // namespace koocadcam::skill
