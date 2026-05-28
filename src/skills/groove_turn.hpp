#pragma once
// @lat: [[engine/skills#groove_turn]]
//
// groove_turn — cut an annular groove (channel) into the outer cylinder.
//
//                ┌─────────────────┐  R_outer
//                │                 │
//        ────────┘    ┌──────┐     └──────── ← center_z + width/2
//                     │      │              groove
//                     │      │              depth = R_outer − R_groove
//        ─────────────┘      └─────────────  ← center_z − width/2
//                     │
//                     │ width
//                     │
//
// Synthesis: annular ring with outer R = current outer, inner R = R_outer −
// depth, over Z range [center_z − width/2, center_z + width/2].  Then cut.
//
// Signature pattern:
//   - 1 cylindrical face at smaller radius (= bbox outer − depth)
//   - 2 annular planar walls perpendicular to Z, one at center_z−width/2
//     (normal -Z) and one at center_z+width/2 (normal +Z)
//
// DFM:
//   DFM-INPUT  : width_mm ≥ 0.5 (parting/grooving insert minimum kerf)
//   DFM-INPUT  : depth_mm > 0 and < bbox.outerRadiusXY() − 1.0
//                 (keep at least 1 mm wall at the bottom of the groove)

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace groove_turn {

constexpr const char* kSkillId = "groove_turn";

struct Input
{
    double  center_z_mm = 0.0;   // groove center Z
    double  width_mm    = 0.0;   // axial width
    double  depth_mm    = 0.0;   // radial depth (outer → inward)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace groove_turn
}  // namespace koocadcam::skill
