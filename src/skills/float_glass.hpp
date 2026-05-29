#pragma once
// @lat: [[engine/skills#float_glass]]
//
// float_glass — produce a continuous flat sheet by floating a glass ribbon
// across a bath of molten tin (Pilkington process).
//
// Molten glass at ~ 1050 °C is fed onto the tin bath; gravity, surface
// tension, and roll-driven tractive forces stretch and thin the ribbon to
// the target `glass_thickness_mm`.  Ribbon speed (`ribbon_speed_m_min`)
// trades thickness against output: thicker ribbons run slower.  The exit
// ribbon is cooled in the lehr to anneal, then scored and snapped to size.
// At the modeling layer, the workpiece envelope IS the flat sheet; we
// treat geometry as a no-op and record the process parameters.
//
//        molten glass ──▶ ┃══════════════════════════════════════┃ ──▶ scored sheet
//                          ▲      tin bath (1050 → 600 °C)       ▲
//                          │       ribbon glides on tin          │
//                          └─────────────────────────────────────┘
//
// Signature pattern:
//   pattern.kind             == "float_glass"
//   pattern.glass_thickness_mm, pattern.ribbon_speed_m_min
//   pattern.geometric_change == false
//
// DFM checks (errors block apply):
//   DFM-FLOAT-THICK : glass_thickness_mm ∈ [0.4, 25.0]
//   DFM-FLOAT-SPEED : ribbon_speed_m_min ∈ (0, 50]
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace float_glass {

constexpr const char* kSkillId = "float_glass";

struct Input
{
    double  glass_thickness_mm  = 4.0;
    double  ribbon_speed_m_min  = 12.0;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace float_glass
}  // namespace koocadcam::skill
