#pragma once
// @lat: [[engine/skills#composite_layup]]
//
// composite_layup — lay alternating plies of fiber + resin into a mold.
//
// The macroscopic geometry of the part (the mold cavity) is NOT changed by
// this operation — the workpiece shape passed in is assumed to BE the final
// laminate envelope.  The skill stamps the workpiece with the laminate's
// engineering metadata (ply count, ply thickness, orientation sequence,
// matrix + fiber material) so downstream skills (composite_cure,
// resin_transfer_mold) and the process planner can validate stack-up
// rules (symmetry, balance, total CPT) and emit cure cycles.
//
//        ply N (90°)     ──────────────────────
//        ply N-1 (0°)    ──────────────────────  ▲ ply_thickness_mm
//        ply N-2 (45°)   ──────────────────────  │
//        ply N-3 (-45°)  ──────────────────────  ▼
//        ...
//
// Signature pattern:
//   pattern.kind            == "composite_layup"
//   pattern.ply_count, pattern.ply_thickness_mm,
//   pattern.orientation_deg [],  pattern.matrix_material,
//   pattern.fiber_material, pattern.symmetric (computed),
//   pattern.balanced  (computed), pattern.cpt_mm (total thickness)
//   pattern.geometric_change == false
//
// DFM checks:
//   DFM-LAYUP-PLIES    : ply_count ≥ 1
//   DFM-LAYUP-THICK    : ply_thickness_mm ∈ [0.05, 0.5] mm
//   DFM-LAYUP-ORIENT   : orientation_deg.size() == ply_count
//   DFM-LAYUP-RANGE    : every orientation ∈ [-90, 90] deg
//   DFM-LAYUP-SYM      : info-only if stack-up not symmetric
//   DFM-LAYUP-BAL      : info-only if not balanced (counts of +θ vs -θ differ)
//   DFM-INPUT          : matrix/fiber material empty
//
// Recognize: metadata-only.  An ideal B-Rep laminate envelope gives no
// geometric clue to fiber orientation.  Returns replayed features.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace composite_layup {

constexpr const char* kSkillId = "composite_layup";

struct Input
{
    int                  ply_count          = 1;
    double               ply_thickness_mm   = 0.15;    // typical CPT for woven prepreg
    std::vector<double>  orientation_deg    = { 0.0 }; // per-ply fiber angle
    std::string          matrix_material    = "epoxy";
    std::string          fiber_material     = "carbon_t300";
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace composite_layup
}  // namespace koocadcam::skill
