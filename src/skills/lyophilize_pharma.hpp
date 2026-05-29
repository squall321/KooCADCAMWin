#pragma once
// @lat: [[engine/skills#lyophilize_pharma]]
//
// lyophilize_pharma — pharmaceutical freeze-drying for parenteral vials,
// biologics, and protein products.  Distinct from the generic `lyophilize`
// skill: records BOTH primary AND secondary drying temperatures and the
// chamber vacuum level, which together govern cake structure and residual
// moisture for sterile injectables.
//
//   wet vial     pharma lyo cycle                      dry stable vial
//  ┌──────────┐  ┌──────────────────────────────────┐ ┌──────────────┐
//  │ frozen   │→│ primary dry (T1, vac) → 2nd dry   │→│ porous cake  │
//  │ solute   │  │ (T2, hold) → stopper             │ │ < 1 % H2O    │
//  └──────────┘  └──────────────────────────────────┘ └──────────────┘
//   Geometry idealized as unchanged (porosity is sub-OCCT).
//
// Signature pattern:
//   pattern.kind                   = "lyophilize_pharma"
//   pattern.primary_dry_temp_c     = double  (typical -40 … -10 °C)
//   pattern.secondary_dry_temp_c   = double  (typical 20 … 40 °C)
//   pattern.vacuum_mbar            = double  (typical 0.01 … 0.5 mbar)
//   pattern.geometric_change       = false
//
// DFM:
//   DFM-INPUT             vacuum_mbar ≤ 0 or > 100 (error)
//   DFM-INPUT             secondary_dry_temp_c ≤ primary_dry_temp_c (error)
//   DFM-LYP-PRIM-TEMP     primary_dry_temp_c not in [-60, 0] (info)
//   DFM-LYP-SEC-TEMP      secondary_dry_temp_c > 60 (info — protein denat risk)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lyophilize_pharma {

constexpr const char* kSkillId = "lyophilize_pharma";

struct Input
{
    double  primary_dry_temp_c   = -30.0;    // °C
    double  secondary_dry_temp_c = 25.0;     // °C
    double  vacuum_mbar          = 0.1;      // mbar
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace lyophilize_pharma
}  // namespace koocadcam::skill
