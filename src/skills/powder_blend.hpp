#pragma once
// @lat: [[engine/skills#powder_blend]]
//
// powder_blend — mix two or more elemental / pre-alloyed powders (with
// optional lubricants, binders) to form a feedstock for downstream
// press-and-sinter or MIM.  The blend recipe defines the composition of
// the alloyed sintered part:
//
//    Σ fractions ≡ 1.00     (mass-fraction normalisation enforced)
//
//      ┌─────────┐  ┌─────────┐         ┌──────────────┐
//      │ powder  │  │ powder  │  blend  │  feedstock   │
//      │   A     │+ │   B   ..│ ──────▶ │  (alloyed)   │
//      └─────────┘  └─────────┘         └──────────────┘
//
// Model: the workpiece is a STAND-IN bulk container.  No geometric
// change — the feature records the blend recipe.
//
// DFM:
//   DFM-PB-COMPONENTS : at least 1 component required (typically 2-6).
//   DFM-PB-FRAC       : every fraction in (0, 1]; sum ≈ 1.0 within 1e-3.
//   DFM-PB-MAT-EMPTY  : component material strings must be non-empty.
//
// Signature pattern:
//   pattern.kind                == "powder_blend"
//   pattern.components          == [{material, fraction}, …]
//   pattern.component_count
//   pattern.process_family      == "powder_metallurgy"
//   pattern.geometric_change    == false
//
// Recognize:
//   Metadata-only — replays history.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace powder_blend {

constexpr const char* kSkillId = "powder_blend";

struct Component
{
    std::string  material;
    double       fraction = 0.0;     // mass fraction; sum must ≈ 1.0
};

struct Input
{
    std::vector<Component>  components;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace powder_blend
}  // namespace koocadcam::skill
