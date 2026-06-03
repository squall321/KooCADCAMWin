#pragma once
// @lat: [[engine/skills#miter_corner_trim]]
//
// miter_corner_trim — Trim a workpiece (typically two intersecting
// structural members fused into a single body) at the bisecting plane of
// two member axes.  For orthogonal members this produces a SolidWorks-style
// 45° miter corner.
//
// Algorithm:
//   1. unit_a = unit(member_a_axis_dir), unit_b = unit(member_b_axis_dir).
//   2. bisector plane normal = unit(unit_a - unit_b) (orthogonal pair → 45°).
//   3. plane passes through corner_xyz.
//   4. Cut workpiece with the half-space on the −member_b side.
//
// Signature pattern:
//   pattern.kind             = "miter_corner_trim"
//   pattern.is_compound      = true
//   pattern.profile_kind     = "miter"
//   pattern.derived_volume_mm3
//
// DFM:
//   DFM-INPUT  : member_a_axis_dir, member_b_axis_dir must be non-zero.
//   DFM-PARA   : axes must not be parallel (need an actual corner).
//   DFM-NULL   : workpiece must be non-null.

#include "Skill.hpp"

#include <array>

namespace koocadcam::skill {

class Workpiece;

namespace miter_corner_trim {

constexpr const char* kSkillId = "miter_corner_trim";

struct Input
{
    std::array<double, 3>  corner_xyz         = { 0.0, 0.0, 0.0 };
    std::array<double, 3>  member_a_axis_dir  = { 1.0, 0.0, 0.0 };
    std::array<double, 3>  member_b_axis_dir  = { 0.0, 1.0, 0.0 };
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace miter_corner_trim
}  // namespace koocadcam::skill
