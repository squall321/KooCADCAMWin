#pragma once
// @lat: [[engine/skills#loft_between_profiles]]
//
// loft_between_profiles — SolidWorks-style "Lofted Boss/Base" feature.
// Skin a smooth solid through ≥ 2 closed cross-sections at different Z
// heights, using BRepOffsetAPI_ThruSections.
//
// Signature pattern:
//   pattern.kind                = "loft_between_profiles"
//   pattern.is_compound         = true
//   pattern.section_count
//   pattern.path_length_mm      (= |z_top − z_bottom|)
//   pattern.derived_volume_mm3
//
// DFM:
//   DFM-INPUT       : ≥ 2 sections required.
//   DFM-LOFT-CLOSED : every section must have ≥ 3 distinct vertices.
//   DFM-LOFT-Z      : section_z_positions size must equal sections size.

#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace loft_between_profiles {

constexpr const char* kSkillId = "loft_between_profiles";

struct Input
{
    // sections[k] = closed XY polyline (in face-local plane at z = section_z_positions[k])
    std::vector<std::vector<std::pair<double,double>>>  sections;
    std::vector<double>                                 section_z_positions;
    bool                                                is_solid = true;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace loft_between_profiles
}  // namespace koocadcam::skill
