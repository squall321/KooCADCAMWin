#pragma once
// @lat: [[engine/skills#lost_wax_jewelry]]
//
// lost_wax_jewelry — investment casting for jewelry-scale parts.  A
// wax pattern (carved or 3D-printed) is sprued, invested in plaster,
// burnt out, and replaced by molten precious metal under centrifugal
// or vacuum-assist pressure.  Metadata-only — the resulting ring /
// pendant / band B-rep was already produced upstream; this skill stamps
// the casting record for traceability and sprue-yield planning.
//
//        ┌─────────────┐                       ┌─────────────┐
//        │ wax pattern │  lost_wax_jewelry     │  cast piece │
//        │  (carved)   │ ────────────────────▶│  (Au/Ag/Pt) │
//        │             │ (alloy, wax_g, sprues)│             │
//        └─────────────┘                       └─────────────┘
//
// Signature pattern:
//   pattern.kind             = "lost_wax_jewelry"
//   pattern.alloy            = "Au18K" | "Au14K" | "Ag925" | "Pt950" | "Pd950" | …
//   pattern.wax_weight_g     = double   (pattern + sprue tree wax mass)
//   pattern.sprue_count      = int      (# of feed channels on tree)
//   pattern.expected_metal_g = double   (computed from alloy density)
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT           alloy empty (error)
//   DFM-WAX-WEIGHT      wax_weight_g ≤ 0 (error)
//   DFM-SPRUE-COUNT     sprue_count ≤ 0 (error)
//   DFM-WAX-WEIGHT-LO   wax_weight_g < 0.05 g — micro-pattern, vacuum-assist needed (info)
//   DFM-WAX-WEIGHT-HI   wax_weight_g > 200 g — large tree, switch to gravity pour (info)
//   DFM-SPRUE-MANY      sprue_count > 24 — overcrowded tree, yield risk (info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lost_wax_jewelry {

constexpr const char* kSkillId = "lost_wax_jewelry";

struct Input
{
    std::string  alloy         = "Au18K";   // Au18K | Au14K | Ag925 | Pt950 | Pd950 | …
    double       wax_weight_g  = 5.0;
    int          sprue_count   = 4;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace lost_wax_jewelry
}  // namespace koocadcam::skill
