#pragma once
// @lat: [[engine/skills#body_split_by_plane]]
//
// body_split_by_plane — Split workpiece into two solids along an arbitrary
// plane defined by (origin, normal).  SolidWorks "Split" operation.
//
// Algorithm:
//   1. Build a half-space box H1 large enough to cover the workpiece on the
//      +normal side of the plane.
//   2. Build a complementary half-space box H2 on the -normal side.
//   3. solidA = wp ∩ H1   (BRepAlgoAPI_Common via Cut against ~H1)
//      solidB = wp ∩ H2
//   4. Return a TopoDS_Compound { solidA, solidB }.
//
// Practical approach using existing primitives:  cut(wp, H_minus)
// → upper part; cut(wp, H_plus) → lower part; pack both into compound.
//
// Signature pattern:
//   - kind = "body_split_by_plane"
//   - is_compound = true
//   - plane_origin / plane_normal
//   - 2 solid sub-shapes in the result compound
//
// DFM:
//   DFM-INPUT  : plane_normal must be non-zero (degenerate plane)
//   DFM-INPUT  : the plane must intersect the bbox (otherwise no split)

#include "Skill.hpp"

#include <array>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace body_split_by_plane {

constexpr const char* kSkillId = "body_split_by_plane";

struct Input
{
    std::array<double,3> plane_origin = { 0.0, 0.0, 0.0 };
    std::array<double,3> plane_normal = { 0.0, 0.0, 1.0 };
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace body_split_by_plane
}  // namespace koocadcam::skill
