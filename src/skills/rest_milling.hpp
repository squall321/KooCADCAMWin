#pragma once
// @lat: [[engine/skills#rest_milling]]
//
// rest_milling — cleanup operation that REMOVES MATERIAL LEFT BEHIND by a
// larger previous tool.  The previous (rougher) tool's radius prevented it
// from reaching into sharp pocket corners; a smaller cleanup-tool comes
// back and finishes those corners.
//
// In slice-1 we model "rest material in the corners" geometrically as
// four small drilled holes at the corners of the input rectangle.  Each
// drill goes to the pocket-bottom depth.  The rough-pocket and the
// pocket-floor connecting them are NOT re-cut by this skill — we assume
// the previous tool already removed the bulk material; rest_milling only
// touches up the four uncut corners.
//
//             ┌──────────────────────────┐
//             │ ○                       ○ │     ○ = cleanup_tool drill
//             │                          │       at each rect corner
//             │                          │       (replicates "tool reaches
//             │     (existing pocket,    │        into corner")
//             │      already roughed)    │
//             │                          │
//             │ ○                       ○ │
//             └──────────────────────────┘
//                          ▲
//                      center_x, center_y
//
// Signature pattern:
//   - 4 small-diameter cylindrical features at the corners
//   - signature.skill_id == "rest_milling"
//   - signature.pattern.kind == "rest_milling"
//   - records previous_tool_dia_mm + cleanup_tool_dia_mm for CAM ordering
//
// DFM checks:
//   DFM-REST-DIA  : cleanup_tool_dia_mm < previous_tool_dia_mm
//                   (rest milling only makes sense with a SMALLER tool)
//   DFM-002       : cleanup_tool_dia_mm ≥ 0.8 mm (end-mill min)
//   geometry sanity (length/width/depth > 0)
//
// Recognize:
//   Similar topology to pocket_with_corner_relief — four small cylindrical
//   features at the rectangle corners.  We differentiate from
//   pocket_with_corner_relief by:
//     * absence of the surrounding rect-pocket walls (rest_milling does
//       NOT re-cut the pocket itself), OR
//     * presence of the cleanup_tool_dia_mm parameter on a previously-
//       stamped FeatureSignature.
//   We prefer the feature-history path; geometric recognition is best-
//   effort with confidence 0.70.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rest_milling {

constexpr const char* kSkillId = "rest_milling";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm           = 0.0;
    double      center_y_mm           = 0.0;
    gp_Dir      axis_dir              { 0.0, 0.0, -1.0 };
    double      length_mm             = 0.0;
    double      width_mm              = 0.0;
    double      depth_mm              = 0.0;
    // Tool that previously roughed the pocket; informs CAM ordering only.
    double      previous_tool_dia_mm  = 0.0;
    // The smaller tool used to clean up the corners.
    double      cleanup_tool_dia_mm   = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rest_milling
}  // namespace koocadcam::skill
