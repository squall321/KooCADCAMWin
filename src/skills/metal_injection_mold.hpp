#pragma once
// @lat: [[engine/skills#metal_injection_mold]]
//
// metal_injection_mold (MIM) — inject a feedstock of fine metal powder
// (typically < 20 µm median particle size) mixed with a polymeric binder
// into a closed mold, debind the green part, then sinter to near-full
// density.  Combines plastic-injection complexity with powder-metallurgy
// material properties.
//
//      ┌────────────┐  inject   ┌────────────┐  debind+sinter  ┌──────────┐
//      │ feedstock  │  ──────▶  │ green part │   ────────────▶ │ MIM part │
//      │ powder+    │           │ (oversized │   shrinks ~15-20│ near-net │
//      │ binder     │           │  by S%)    │   %             │ shape    │
//      └────────────┘           └────────────┘                 └──────────┘
//
// Model: the input workpiece IS the as-sintered final part.  MIM is a
// PROCESS that records intent — material, binder %, expected sinter
// shrinkage — without mutating geometry.  Process planners downstream use
// the recorded shrink factor to PRE-SCALE the actual tool cavity at
// CAM-time (cavity_dim = part_dim / (1 − shrink_pct/100)); the workpiece
// representation here is the final geometry.
//
// DFM:
//   DFM-MIM-WALL      : wall_thickness ≥ 0.5 mm (MIM minimum; finer walls
//                       collapse during sinter).
//   DFM-MIM-WALL-HEAVY: wall_thickness > 10 mm → debinding cracking risk
//                       (info-only — MIM is best for thin sections).
//   DFM-MIM-BINDER    : binder_pct in [30, 50] (volume).  < 30 % no flow;
//                       > 50 % excessive shrinkage / slump.
//   DFM-MIM-SHRINK    : sinter_shrinkage_pct in [10, 25] typical envelope.
//   DFM-MIM-MAT       : material in MIM catalog (info-only on unknowns).
//
// Signature pattern:
//   pattern.kind                       == "metal_injection_mold"
//   pattern.material, pattern.binder_pct, pattern.sinter_shrinkage_pct
//   pattern.predicted_cavity_scale     (1 / (1 − S/100))
//   pattern.process_family             == "powder_metallurgy"
//   pattern.process_subtype            == "mim"
//   pattern.geometric_change           == false
//   pattern.additive_workflow          == true
//
// Recognize:
//   Metadata-only — replays history if present.  MIM parts cannot be
//   distinguished from machined billet parts by B-rep topology alone.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace metal_injection_mold {

constexpr const char* kSkillId = "metal_injection_mold";

struct Input
{
    std::string  material              = "stainless_316";   // any MIM feedstock
    double       binder_pct            = 40.0;              // volume %
    double       sinter_shrinkage_pct  = 17.0;              // linear shrink %
    double       wall_thickness_mm     = 2.0;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace metal_injection_mold
}  // namespace koocadcam::skill
