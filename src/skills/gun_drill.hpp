#pragma once
// @lat: [[engine/skills#gun_drill]]
//
// gun_drill — deep, straight, precision drilling using a gun drill (single
// flute + coolant-through-tool).  Used where standard twist drills wander
// off-axis due to high depth/diameter ratios.
//
// Geometrically identical to a deep drill_hole.  Differentiated from drill_hole
// via:
//   - signature.pattern.kind         = "gun_drill"
//   - tooling.tool_type              = "gun_drill"
//   - tooling.flute_count            = 1               (vs 2 for twist drill)
//   - tooling.extra.through_coolant  = true
//   - tooling.extra.straightness_class
//
//                          ┌─ entry_face (planar)
//                          ▼
//                    ──────┬─┬──────
//                          │ │  ▼ axis
//                          │ │           ↑ depth_mm (deep — depth/dia ≥ 10)
//                          │ │           │
//                          │ │           │
//                          │ │           ↓
//                          └─┘
//                           ↑
//                          dia_mm
//
// Signature pattern:
//   - 1 cylindrical face (the bore wall)
//   - 1 circular edge at the entry face
//   - 1 circular edge at the blind bottom + 1 planar bottom face
//   - depth/diameter ratio ≥ 10
//
// DFM checks:
//   DFM-GUNDRILL-DIAMIN  : dia ≥ 1.0 mm    (gun drill minimum)
//   DFM-GUNDRILL-DIAMAX  : dia ≤ 50.0 mm   (gun drill maximum)
//   DFM-GUNDRILL-RATIO   : depth/dia between 10 and 100
//
// Recognize:
//   Cylindrical face with depth/dia ratio ≥ 10.  Confidence 0.6 (overlaps
//   with drill_hole and deep_hole_peck recognition).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gun_drill {

constexpr const char* kSkillId = "gun_drill";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm     = 0.0;
    double      position_y_mm     = 0.0;
    gp_Dir      axis_dir          { 0.0, 0.0, -1.0 };
    double      diameter_mm       = 0.0;
    double      depth_mm          = 0.0;
    bool        through_hole      = false;
    std::string straightness_class = "0.05_per_100mm";   // typical class
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gun_drill
}  // namespace koocadcam::skill
