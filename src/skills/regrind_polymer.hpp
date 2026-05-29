#pragma once
// @lat: [[engine/skills#regrind_polymer]]
//
// regrind_polymer — polymer regrinding for re-use.  Sorted polymer fractions
// (from separator_sortation) are reduced to flake / pellet size, optionally
// contaminated with foreign material that survived sortation.  Records the
// polymer type, the final grind particle size, and the residual
// contamination level.
//
//        ┌─────────────┐                       ┌─────────────┐
//        │ polymer frac│  regrind_polymer      │  regrind    │
//        │  ▓▓▓▓▓▓▓▓▓  │ ────────────────▶    │  (sig only) │
//        │             │  (type, size, cont%)  │  flakes:    │
//        └─────────────┘                       │  ▒ ▒ ▒ ▒ ▒ │
//                                              └─────────────┘
//
// Signature pattern:
//   - pattern.kind                = "regrind_polymer"
//   - pattern.polymer_type        = "PET" | "PP" | "HDPE" | "LDPE" | "PS" | "ABS" | ...
//   - pattern.grind_size_mm       = nominal flake / pellet size (mm)
//   - pattern.contamination_pct   = residual non-polymer fraction (%)
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   grind_size_mm > 0                  (DFM-INPUT error if not)
//   contamination_pct ∈ [0, 100)       (DFM-INPUT error if not)
//   polymer_type non-empty             (DFM-INPUT error if empty)
//   polymer_type recognized            (DFM-REGRIND-TYPE info if not)
//   grind_size_mm < 1                  (DFM-REGRIND-FINE info — very fine
//                                       powder; likely energy-intensive,
//                                       cryo-grinding may be needed)
//   grind_size_mm > 25                 (DFM-REGRIND-COARSE info — exceeds
//                                       feed-throat of typical extruder)
//   contamination_pct > 5              (DFM-REGRIND-CONT warning —
//                                       high foreign-material fraction
//                                       degrades re-melt quality)
//
// Synthesis:
//   NO geometric change.  Clone the workpiece (shape + material) + stamp.
//
// Recognize:
//   Impossible from B-rep alone — signature-replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace regrind_polymer {

constexpr const char* kSkillId = "regrind_polymer";

struct Input
{
    std::string  polymer_type        = "PET";    // PET | PP | HDPE | LDPE | PS | ABS
    double       grind_size_mm       = 5.0;
    double       contamination_pct   = 1.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace regrind_polymer
}  // namespace koocadcam::skill
