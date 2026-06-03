#pragma once
// @lat: [[engine/skills#side_core_pull_slot]]
//
// side_core_pull_slot — Cut a rectangular slot in a mold face for a
// side-action core (a slide perpendicular to the main mold-pull direction).
// The slot also includes a retract_clearance pocket so the side core can
// pull out cleanly, and the slot side walls are drafted at 3° for release.
//
//                       pull_dir_xyz  (side direction, NOT main pull)
//                       ────►
//      face (mold)  ┌───────────┐
//                   │   slot    │  slot_width × slot_height
//                   │   pocket  │  slot_depth into face along pull_dir
//                   └───────────┘
//                   ▲
//                   retract_clearance_mm
//
// Implementation:
//   1. Find the face center; build slot box (slot_width × slot_height ×
//      slot_depth + retract_clearance) centred in-plane on the face_id face
//      and oriented so its depth-axis matches pull_dir_xyz.
//   2. Cut from workpiece.
//
// Signature pattern:
//   pattern.kind                    = "side_core_pull_slot"
//   pattern.is_compound             = true
//   pattern.mold_feature_type       = "side_core_slot"
//   pattern.derived_volume_removed  = slot volume
//
// DFM:
//   DFM-INPUT             : width/height/depth > 0
//   DFM-SCORE-CLEARANCE   : retract_clearance ≥ 0.5 mm
//   DFM-SCORE-DRAFT       : draft 3° applied internally; refuse if input
//                           dims so small drafted top would degenerate.

#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace side_core_pull_slot {

constexpr const char* kSkillId = "side_core_pull_slot";

struct Input
{
    int     face_id              = -1;     // mold face where side core enters
    double  pull_dir_x           = 1.0;    // side core retract direction
    double  pull_dir_y           = 0.0;
    double  pull_dir_z           = 0.0;
    double  slot_width_mm        = 0.0;
    double  slot_height_mm       = 0.0;
    double  slot_depth_mm        = 0.0;
    double  retract_clearance_mm = 1.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace side_core_pull_slot
}  // namespace koocadcam::skill
