#pragma once
// @lat: [[engine/skills#copy_body_with_transform]]
//
// copy_body_with_transform — Duplicate workpiece with a translation+rotation,
// then return a Compound { original, copy }.  SolidWorks "Move/Copy Body".
//
// Transform:
//   T = Translation(translation_xyz) ∘ Rotation(axis_origin, axis_dir, angle)
//
// Algorithm:
//   1. Build gp_Trsf with rotation about (axis_origin, axis_dir) by angle.
//   2. Multiply by translation gp_Trsf.
//   3. BRepBuilderAPI_Transform(wp.shape(), T).Shape() → copy
//   4. Pack { original, copy } into a TopoDS_Compound.
//
// Signature pattern:
//   - kind = "copy_body_with_transform"
//   - is_compound = true
//   - translation_xyz, rotation_axis_origin, rotation_axis_dir, rotation_angle_deg
//
// DFM:
//   DFM-INPUT  : rotation axis direction must be non-zero
//   DFM-INPUT  : transform must be non-identity (translation OR angle ≠ 0)

#include "Skill.hpp"

#include <array>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace copy_body_with_transform {

constexpr const char* kSkillId = "copy_body_with_transform";

struct Input
{
    std::array<double,3> translation_xyz       = { 0.0, 0.0, 0.0 };
    std::array<double,3> rotation_axis_origin  = { 0.0, 0.0, 0.0 };
    std::array<double,3> rotation_axis_dir     = { 0.0, 0.0, 1.0 };
    double               rotation_angle_deg    = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace copy_body_with_transform
}  // namespace koocadcam::skill
