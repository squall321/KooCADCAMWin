#pragma once
// @lat: [[engine/skills#cell_seed]]
//
// cell_seed — deposit (seed) live cells onto an already-fabricated scaffold.
//
//        scaffold (pre-fab)              after cell_seed
//        ┌─────────────┐                 ┌─────────────┐
//        │ ░░░░░░░░░░░ │   pipette       │ ●●●●●●●●●●● │  ← attached cells
//        │  scaffold   │  ──────────▶   │  scaffold   │
//        │  matrix     │   cells + media │  + cells    │
//        └─────────────┘                 └─────────────┘
//
// Distinguished from `bioink_print`: cell_seed *adds cells to* an existing
// scaffold (post-fabrication), whereas bioink_print extrudes cell-laden
// hydrogel as part of the build itself.  Geometry of the scaffold is
// preserved — only metadata records the seeding intent.
//
// Signature pattern:
//   pattern.kind                 = "cell_seed"
//   pattern.cell_type            = "fibroblast" | "hepatocyte" | "msc" | …
//   pattern.density_cells_cm2    = double  (areal seeding density)
//   pattern.viability_pct        = double  (initial post-seed viability, %)
//   pattern.geometric_change     = false
//
// DFM:
//   DFM-INPUT              cell_type empty
//   DFM-SEED-DENSITY       density_cells_cm2 ∈ [1e2, 1e7] cells/cm²
//   DFM-SEED-VIABILITY     viability_pct ∈ [0, 100]
//   DFM-SEED-VIABILITY-LOW viability_pct < 70 % (info — culture rescue risk)
//
// Recognize: metadata-only — analytic B-Rep cannot detect attached cells.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cell_seed {

constexpr const char* kSkillId = "cell_seed";

struct Input
{
    std::string  cell_type          = "fibroblast";
    double       density_cells_cm2  = 1.0e5;     // cells / cm²
    double       viability_pct      = 95.0;      // %
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace cell_seed
}  // namespace koocadcam::skill
