#pragma once
// @lat: [[engine/skills#scale_body_uniform]]
//
// scale_body_uniform — Uniformly scale the workpiece around a centre point.
// Equivalent to SolidWorks "Scale" body operation with uniform factor.
//
//                          factor 0.5         factor 2.0
//          ┌────────┐         ┌────┐         ┌────────────────┐
//          │  body  │   ───►  │ bd │   ───►  │      body      │
//          │        │         └────┘         │                │
//          └────────┘                        └────────────────┘
//
// Algorithm:
//   1. trsf.SetScale(gp_Pnt(center), factor).
//   2. BRepBuilderAPI_Transform(wp.shape(), trsf, /*copy*/ true).
//
// Signature pattern:
//   pattern.kind             = "scale_body_uniform"
//   pattern.is_compound      = true
//   pattern.parameters       = { center_xyz, scale_factor }
//   pattern.derived_volume   = original_volume × factor^3
//
// DFM:
//   DFM-SCALE-RANGE  : factor ∈ [0.5, 3.0].

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace scale_body_uniform {

constexpr const char* kSkillId = "scale_body_uniform";

struct Input
{
    std::array<double, 3>  center_xyz   { 0.0, 0.0, 0.0 };
    double                 scale_factor = 1.0;     // expected ∈ [0.5, 3.0]
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace scale_body_uniform
}  // namespace koocadcam::skill
