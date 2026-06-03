#pragma once
// @lat: [[engine/skills#lifter_mechanism]]
//
// lifter_mechanism — Cut an angled prismatic channel representing the
// envelope of a mold lifter — an angled ejector rod that retracts
// diagonally during ejection to release internal undercuts.
//
//        lifter_angle_deg (from vertical, 0-15°)
//          ▲
//          │
//          │   lifter envelope = tilted box
//          │   (lifter_width × lifter_height × lift_stroke)
//      ────┘────  mold face
//      base_xy
//
// Implementation:
//   1. Build an axis-aligned box (width × height × stroke).
//   2. Rotate by lifter_angle_deg about an in-plane axis through base_xy.
//   3. Cut from workpiece.
//
// Signature pattern:
//   pattern.kind                   = "lifter_mechanism"
//   pattern.is_compound            = true
//   pattern.mold_feature_type      = "lifter_channel"
//   pattern.derived_volume_removed = width × height × stroke
//
// DFM:
//   DFM-INPUT          : width/height/stroke > 0
//   DFM-LIFTER-ANGLE   : angle in [0, 15] deg
//   DFM-LIFTER-CLEAR   : stroke ≥ 0.5 mm

#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace lifter_mechanism {

constexpr const char* kSkillId = "lifter_mechanism";

struct Input
{
    int     face_id           = -1;    // mold face that hosts the lifter
    double  base_x_mm         = 0.0;
    double  base_y_mm         = 0.0;
    double  lifter_angle_deg  = 5.0;   // 0-15
    double  lifter_width_mm   = 0.0;
    double  lifter_height_mm  = 0.0;
    double  lift_stroke_mm    = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lifter_mechanism
}  // namespace koocadcam::skill
