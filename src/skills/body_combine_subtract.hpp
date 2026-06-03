#pragma once
// @lat: [[engine/skills#body_combine_subtract]]
//
// body_combine_subtract — Boolean MINUS (workpiece - aux body).
// SolidWorks "Combine → Subtract" operation.
//
// Auxiliary shape is built from primitives identified by aux_kind:
//   "box"      — pr::box(gp_Ax2(origin, gp::DZ()), dx, dy, dz)
//   "cylinder" — pr::cylinder(gp_Ax2(origin, gp::DZ()), radius, height)
//
// Signature pattern:
//   - kind = "body_combine_subtract"
//   - is_compound = false
//   - aux_kind, aux_origin, aux_dims
//   - derived_removed_volume_mm3
//
// DFM:
//   DFM-INPUT  : aux_kind ∈ {"box", "cylinder"}
//   DFM-INPUT  : all dims must be > 0

#include "Skill.hpp"

#include <array>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace body_combine_subtract {

constexpr const char* kSkillId = "body_combine_subtract";

struct Input
{
    std::string          aux_kind   = "box";
    std::array<double,3> aux_origin = { 0.0, 0.0, 0.0 };
    std::array<double,3> aux_dims   = { 10.0, 10.0, 10.0 };
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace body_combine_subtract
}  // namespace koocadcam::skill
