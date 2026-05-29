#pragma once
// @lat: [[engine/skills#glass_blow_mold]]
//
// glass_blow_mold — form a hollow glass container by blowing a gob (parison)
// against the interior of a steel/cast-iron mold.
//
// A measured parison of viscous molten glass is delivered into the
// blank/parison mold, given a counter-blow puff to establish a thick wall,
// then transferred to the finish (blow) mold where compressed air at
// `blow_pressure_kpa` inflates the parison to the mold's internal cavity.
// The workpiece envelope IS the finished container outer surface; the
// modeling layer treats geometry as a no-op (caller supplies the shape)
// and records process parameters as metadata.
//
//        ┌──────────┐  counter-blow   ┌──────────┐    final blow    ┌──────────┐
//        │ parison  │  ────────────▶ │ pre-form │  ──────────────▶│ bottle   │
//        │ (gob)    │  (blank mold)   │          │  (finish mold)   │          │
//        └──────────┘                 └──────────┘                  └──────────┘
//
// Signature pattern:
//   pattern.kind             == "glass_blow_mold"
//   pattern.mold_id, pattern.parison_weight_g, pattern.blow_pressure_kpa
//   pattern.geometric_change == false   (envelope provided by caller)
//
// DFM checks (errors block apply):
//   DFM-BLOW-MOLD     : mold_id non-empty
//   DFM-BLOW-PARISON  : parison_weight_g ∈ (0, 5000]
//   DFM-BLOW-PRESSURE : blow_pressure_kpa ∈ [50, 500] kPa
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace glass_blow_mold {

constexpr const char* kSkillId = "glass_blow_mold";

struct Input
{
    std::string  mold_id           = "BM-330ml-v1";
    double       parison_weight_g  = 220.0;
    double       blow_pressure_kpa = 200.0;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace glass_blow_mold
}  // namespace koocadcam::skill
