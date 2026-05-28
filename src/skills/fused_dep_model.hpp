#pragma once
// @lat: [[engine/skills#fused_dep_model]]
//
// fused_dep_model — Fused Deposition Modeling (FDM) with explicit NOZZLE
// geometry knobs.  An alias for the FDM variant of 3d_print, but
// parameterised by the physical printer settings (nozzle diameter, print
// speed, entry face datum).  Lets process planners reason about
// nozzle/layer compatibility, single-line wall widths, and layer adhesion
// without having to thread those numbers through the generic 3d_print
// envelope.
//
//                              filament
//                                  │
//                                  ▼
//                              ╔═══════╗  nozzle_dia_mm
//                              ║  ▼ ▼ ▼ ║
//                              ╚═══┬═══╝
//                                  │  ← extrusion bead, width ≈ nozzle_dia
//          layer N+1  ────────────────────────────  ▲
//          layer N    ────────────────────────────  │ layer_height_mm
//          layer N−1  ────────────────────────────  ▼     (≤ 0.8 × nozzle_dia)
//
// vs 3d_print: same outcome (no geometric change to the workpiece — the
// shape IS the final printed part) but DFM is tighter: layer height must
// not exceed 80% of the nozzle diameter, otherwise inter-layer bonds are
// weak (the bead lacks enough overlap to mechanically interlock).
//
// DFM:
//   DFM-FDM-NOZ      : nozzle_dia_mm ∈ [0.2, 1.0]
//   DFM-FDM-LAYER    : layer_height_mm ≤ nozzle_dia × 0.8
//                      (else weak inter-layer bond → ERROR, not a warning)
//   DFM-FDM-SPEED    : print_speed_mm_per_s > 0
//
// Signature pattern:
//   pattern.kind == "fused_dep_model"
//   pattern.nozzle_dia_mm, pattern.layer_height_mm,
//   pattern.material, pattern.print_speed_mm_per_s
//   pattern.geometric_change == false
//
// Recognize: metadata-only (same rationale as 3d_print).

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace fused_dep_model {

constexpr const char* kSkillId = "fused_dep_model";

struct Input
{
    FaceDatum     entry_face;                          // build-plate / first-layer reference face
    double        nozzle_dia_mm        = 0.4;
    double        layer_height_mm      = 0.2;          // typically nozzle × 0.5
    std::string   material             = "pla";
    double        print_speed_mm_per_s = 60.0;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace fused_dep_model
}  // namespace koocadcam::skill
