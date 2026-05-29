#pragma once
// @lat: [[engine/skills#lyophilize]]
//
// lyophilize — freeze-drying (sublimation removal of frozen water).
//
//   Phase 1 (Freeze)        : product cooled below eutectic / Tg' point
//   Phase 2 (Primary dry)   : pressure reduced; ice sublimates
//   Phase 3 (Secondary dry) : adsorbed water desorbed → residual moisture
//
//     wet product               lyophilize                 dry product
//   ┌─────────────┐         ┌─────────────────┐         ┌─────────────┐
//   │  (water)    │   →    │  freeze + vac    │   →    │  porous     │
//   │  scaffold   │         │  primary + 2nd   │         │  matrix     │
//   └─────────────┘         └─────────────────┘         └─────────────┘
//   geometry idealized as unchanged (porosity is sub-OCCT)
//
// Signature pattern:
//   pattern.kind                       = "lyophilize"
//   pattern.freeze_temp_c              = double  (typical -80 … -40 °C)
//   pattern.primary_dry_pressure_mbar  = double  (typical 0.01 … 1 mbar)
//   pattern.residual_moisture_pct      = double  (typical 0.5 … 5 %)
//   pattern.geometric_change           = false
//
// DFM:
//   DFM-INPUT                  freeze_temp_c ≥ 0 °C (must be sub-zero)
//   DFM-LYO-PRESSURE           primary_dry_pressure_mbar ∈ (0, 100]
//   DFM-LYO-MOISTURE           residual_moisture_pct ∈ [0, 20]
//   DFM-LYO-MOISTURE-HIGH      residual_moisture_pct > 5 (info — long-term
//                              stability risk for protein products)
//
// Recognize: metadata-only.  Internal porosity from sublimation is below
// the analytic B-Rep modeling layer.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lyophilize {

constexpr const char* kSkillId = "lyophilize";

struct Input
{
    double  freeze_temp_c             = -60.0;   // °C (sub-zero)
    double  primary_dry_pressure_mbar = 0.1;     // mbar
    double  residual_moisture_pct     = 2.0;     // %
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace lyophilize
}  // namespace koocadcam::skill
