#pragma once
// @lat: [[engine/skills#loft_with_guide_curves]]
//
// loft_with_guide_curves — SolidWorks-style "Lofted Boss/Base with Guide".
// Skin a solid through exactly 2 closed cross-sections, with shape
// constrained along a single guide spine. Uses BRepOffsetAPI_MakePipeShell
// with the guide curve as spine and the 2 profiles as auxiliary sections.
//
// Signature pattern:
//   pattern.kind                = "loft_with_guide_curves"
//   pattern.is_compound         = true
//   pattern.section_count       = 2
//   pattern.path_length_mm      (length of guide spine)
//   pattern.derived_volume_mm3
//
// DFM:
//   DFM-INPUT       : exactly 2 sections required.
//   DFM-LOFT-CLOSED : every section must have ≥ 3 distinct vertices.
//   DFM-LOFT-GUIDE  : guide_curve must have ≥ 2 points and length > 0.

#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace loft_with_guide_curves {

constexpr const char* kSkillId = "loft_with_guide_curves";

struct Input
{
    // Exactly 2 closed cross-section polylines (XY plane, positioned at
    // endpoints of guide_curve).
    std::vector<std::vector<std::pair<double,double>>>  sections;
    // Guide spine — a 3D polyline.
    std::vector<gp_Pnt>                                 guide_curve;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace loft_with_guide_curves
}  // namespace koocadcam::skill
