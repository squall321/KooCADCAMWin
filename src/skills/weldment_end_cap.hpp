#pragma once
// @lat: [[engine/skills#weldment_end_cap]]
//
// weldment_end_cap — Closed plate at the end of a tube/structural member.
// Build a profile face perpendicular to axis_dir at axis_origin, extrude by
// cap_thickness along axis_dir, then FUSE onto the workpiece.
//
//                                ┌────────┐  ← cap (thickness)
//                                │        │
//   tube ─────────────────────►  └────────┘
//
// Signature pattern:
//   pattern.kind             = "weldment_end_cap"
//   pattern.is_compound      = true
//   pattern.profile_kind
//   pattern.derived_volume_mm3
//
// DFM:
//   DFM-INPUT  : cap_thickness > 0, profile_dim > 0, axis_dir non-zero.
//   DFM-KIND   : profile_kind ∈ { "square", "round" }.

#include "Skill.hpp"

#include <array>
#include <string>

namespace koocadcam::skill {

class Workpiece;

namespace weldment_end_cap {

constexpr const char* kSkillId = "weldment_end_cap";

struct Input
{
    std::array<double, 3>   tube_axis_origin = { 0.0, 0.0, 0.0 };
    std::array<double, 3>   tube_axis_dir    = { 0.0, 0.0, 1.0 };
    double                  cap_thickness_mm = 5.0;
    std::string             profile_kind     = "square";   // "square" | "round"
    double                  profile_dim_mm   = 50.0;       // side length or diameter
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace weldment_end_cap
}  // namespace koocadcam::skill
