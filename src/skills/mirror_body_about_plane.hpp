#pragma once
// @lat: [[engine/skills#mirror_body_about_plane]]
//
// mirror_body_about_plane — Mirror the entire workpiece about a plane and
// FUSE the mirrored body with the original.  This is SolidWorks "Mirror"
// applied at the body level (additive variant).
//
//                                    │ plane
//                ┌────┐               │              ┌────┐ │ ┌────┐
//                │ A  │   mirror →    │              │ A  │ │ │ A' │
//                └────┘               │              └────┘ │ └────┘
//                                    │
//
// Algorithm:
//   1. trsf.SetMirror(gp_Ax2(origin, normal)).
//   2. BRepBuilderAPI_Transform(wp.shape(), trsf, /*copy*/ true).
//   3. pr::fuse(original, mirrored).
//
// Signature pattern:
//   pattern.kind             = "mirror_body_about_plane"
//   pattern.is_compound      = true
//   pattern.parameters       = { plane_origin, plane_normal }
//   pattern.derived_volume   = volume after fuse (≈ 2 × original)
//
// DFM:
//   DFM-INPUT          : plane_normal must be non-zero.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mirror_body_about_plane {

constexpr const char* kSkillId = "mirror_body_about_plane";

struct Input
{
    std::array<double, 3>  plane_origin { 0.0, 0.0, 0.0 };
    std::array<double, 3>  plane_normal { 1.0, 0.0, 0.0 };
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mirror_body_about_plane
}  // namespace koocadcam::skill
