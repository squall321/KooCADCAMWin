#pragma once
// @lat: [[engine/skills#transformer_wind]]
//
// transformer_wind — winding installation record for a power / distribution
// transformer.  Primary and secondary copper coils are wound on a laminated
// E/I or toroidal core; turns ratio sets voltage ratio.  The macroscopic
// core is unchanged; this skill stamps coil metadata.
//
//                  ┌─────── primary turns N₁ ───────┐
//   AC in ────────►│                                 │────► V₂ = V₁ × N₂/N₁
//                  │                                 │
//                  └─────── secondary turns N₂ ─────┘
//
// Signature pattern:
//   - pattern.kind              = "transformer_wind"
//   - pattern.primary_turns     = N₁
//   - pattern.secondary_turns   = N₂
//   - pattern.wire_dia_mm       = winding-wire Ø
//   - pattern.turns_ratio       = N₁ / N₂   (derived)
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   primary_turns   > 0          (DFM-INPUT error)
//   secondary_turns > 0          (DFM-INPUT error)
//   wire_dia_mm ∈ (0.05, 10.0]   (DFM-XFMR-WIRE error)
//
// Synthesis: no geometric change — clone + stamp.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace transformer_wind {

constexpr const char* kSkillId = "transformer_wind";

struct Input
{
    int    primary_turns   = 200;
    int    secondary_turns = 20;
    double wire_dia_mm     = 1.5;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace transformer_wind
}  // namespace koocadcam::skill
