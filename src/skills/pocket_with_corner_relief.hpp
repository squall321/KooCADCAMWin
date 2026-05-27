#pragma once
// @lat: [[engine/skills#pocket_with_corner_relief]]
//
// pocket_with_corner_relief — COMPOUND MACRO that combines 4 corner-relief
// drill holes + 1 rectangular pocket so the resulting pocket has "sharp"
// (square) corners despite being milled with a round-end-mill.
//
// Workflow:
//   1. Drill 4 holes (one at each pocket corner) deeper than the pocket
//      bottom by 0.5 mm.  Their diameter is corner_relief_dia_mm.
//   2. Mill the rectangular pocket on top of those holes with
//      mill_rect_pocket, using corner_r_mm = corner_relief_dia_mm / 2 so
//      the end-mill's natural fillet exactly inscribes the corner hole.
//
//                ┌──────────────────────────┐
//                │ ○                       ○ │     ○ = corner-relief drill
//                │                          │     (extends 0.5 mm below
//                │                          │      the pocket floor)
//                │      (rect pocket)       │
//                │                          │
//                │                          │
//                │ ○                       ○ │
//                └──────────────────────────┘
//                              ▲
//                          center_x, center_y
//
// Signature pattern:
//   - 4 corner cylindrical-face features (one per drilled relief)
//   - 4 planar walls of the rect pocket
//   - 1 bottom planar face of the rect pocket (with 4 drill holes piercing it)
//   - signature.skill_id == "pocket_with_corner_relief"
//   - signature.pattern.kind == "pocket_with_corner_relief"
//   - signature.pattern.corner_relief_dia_mm carries the relief size so
//     recognize() can find this distinctly from a plain rect pocket.
//
// DFM checks:
//   DFM-002 : corner_relief_dia ≥ 0.8 mm
//   DFM-004 : corner_r (derived) ≥ 0.2 mm → since corner_r = relief_dia/2,
//             that means corner_relief_dia ≥ 0.4 mm (auto-satisfied by
//             the 0.8 mm minimum above, but we keep the explicit check
//             in case the rule changes).
//   length/width/depth > 0.
//
// Recognize:
//   We look for rectangular pockets (delegate to mill_rect_pocket::recognize)
//   that have ADDITIONAL cylindrical features positioned at the 4 corners
//   and extending DEEPER than the pocket bottom.  This is a slice-1
//   heuristic — full BREP adjacency is left for slice-2.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pocket_with_corner_relief {

constexpr const char* kSkillId = "pocket_with_corner_relief";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm           = 0.0;
    double      center_y_mm           = 0.0;
    gp_Dir      axis_dir              { 0.0, 0.0, -1.0 };
    double      length_mm             = 0.0;   // X extent
    double      width_mm              = 0.0;   // Y extent
    double      depth_mm              = 0.0;
    double      corner_relief_dia_mm  = 0.0;   // drill at each corner
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pocket_with_corner_relief
}  // namespace koocadcam::skill
