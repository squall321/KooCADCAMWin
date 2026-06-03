#pragma once
// @lat: [[engine/skills#runner_balanced_h]]
//
// runner_balanced_h — H-pattern balanced runner system feeding 4 or 8 cavities
// from a single sprue.  The geometry is the classic injection-mold H-runner:
//   1 sprue (vertical) + 2 primary legs (along +X / −X) + 4 secondary legs
//   (along +Y / −Y at each primary tip)  → 4 cavities.
// For 8 cavities, the secondary legs branch again into tertiary pairs.
//
//                     ┌─────────┐ ┌─────────┐
//                     │ cavity  │ │ cavity  │
//                     └────┬────┘ └────┬────┘
//                          │           │
//                     ┌────┴───────────┴────┐
//                     │     primary leg     │
//        sprue ●──────┤    sprue point      ├──────●
//                     │                     │
//                     └────┬───────────┬────┘
//                          │           │
//                     ┌────┴────┐ ┌────┴────┐
//                     │ cavity  │ │ cavity  │
//                     └─────────┘ └─────────┘
//
// Implementation:
//   1. Cut a cylindrical sprue at sprue_xy going down to runner_depth_mm.
//   2. Cut 2 primary legs (boxes) extending ±leg_length_mm along X at sprue
//      depth.  Cross-section ≈ trapezoidal approximated by a circular
//      cylinder of `runner_dia_mm`.
//   3. Cut 4 secondary legs perpendicular at primary tips.
//   4. If cavity_count == 8 → also cut 8 tertiary legs at half-length.
//
// Signature pattern:
//   pattern.kind                    = "runner_balanced_h"
//   pattern.is_compound             = true
//   pattern.mold_feature_type       = "balanced_h_runner"
//   pattern.derived_volume_removed  = sum cyl volumes
//
// DFM:
//   DFM-INPUT           : leg_length / runner_dia / runner_depth > 0
//   DFM-RUNNER-DIA      : runner_dia in [2, 8] mm typical
//   DFM-RUNNER-CAVITY   : cavity_count ∈ {4, 8}

#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace runner_balanced_h {

constexpr const char* kSkillId = "runner_balanced_h";

struct Input
{
    double  sprue_x_mm       = 0.0;
    double  sprue_y_mm       = 0.0;
    double  leg_length_mm    = 0.0;     // primary leg half-length
    double  runner_dia_mm    = 0.0;
    double  runner_depth_mm  = 0.0;     // distance the sprue/runner descends
    int     cavity_count     = 4;       // 4 or 8
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace runner_balanced_h
}  // namespace koocadcam::skill
