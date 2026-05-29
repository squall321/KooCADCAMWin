#pragma once
// @lat: [[engine/skills#gravure_print]]
//
// gravure_print — rotogravure printing.  Ink is held in engraved cells on a
// chrome-plated cylinder, transferred to web by direct contact.  Macroscopic
// B-rep is unchanged; printing metadata is stamped onto the workpiece.
//
// Signature pattern:
//   pattern.kind             = "gravure_print"
//   pattern.cylinder_dia_mm  = double (cylinder Ø; 100..800 typical)
//   pattern.line_speed_m_min = double (web speed)
//   pattern.depth_um         = double (cell depth in micrometres)
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             any of cylinder_dia_mm, line_speed_m_min, depth_um ≤ 0 (error)
//   DFM-CYL-SMALL         cylinder_dia_mm < 100 (info — short repeat / niche press)
//   DFM-CYL-LARGE         cylinder_dia_mm > 800 (info — heavy cylinder, special handling)
//   DFM-DEPTH-SHALLOW     depth_um < 5 (info — light tones)
//   DFM-DEPTH-DEEP        depth_um > 60 (info — heavy ink lay-down, drying load)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gravure_print {

constexpr const char* kSkillId = "gravure_print";

struct Input
{
    double cylinder_dia_mm  = 250.0;
    double line_speed_m_min = 300.0;
    double depth_um         = 30.0;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gravure_print
}  // namespace koocadcam::skill
