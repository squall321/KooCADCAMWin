#pragma once
// @lat: [[engine/skills#separator_sortation]]
//
// separator_sortation — magnetic / density / eddy-current sortation of a
// mixed shredded stream into a target fraction (ferrous, non-ferrous,
// polymer, …).  This is a metadata-only step: B-rep is unchanged; the
// signature records WHICH method was used, the target fraction it isolates,
// and the achieved purity (%).
//
//        ┌─────────────┐                       ┌─────────────┐
//        │ mixed feed  │   separator           │ target frac │
//        │ ▒▓░▒▓░▒▓░▒  │ ────────────────▶    │  ▓▓▓▓▓▓▓▓▓  │
//        │ (post-shred)│  method, frac, %      │  (purity %) │
//        └─────────────┘                       └─────────────┘
//
// Signature pattern:
//   - pattern.kind            = "separator_sortation"
//   - pattern.method          = "magnetic" | "density" | "eddy_current"
//   - pattern.target_fraction = "ferrous" | "non_ferrous" | "polymer" | "glass" | ...
//   - pattern.purity_pct      = output purity, [0..100]
//   - pattern.geometry_changed = false
//
// DFM checks:
//   purity_pct ∈ (0, 100]             (DFM-INPUT error if not)
//   method recognized                 (DFM-SORT-METHOD info if not)
//   target_fraction non-empty         (DFM-INPUT error if empty)
//   purity_pct < 90                   (DFM-SORT-PURITY warning —
//                                      downstream furnace may need re-sort)
//   method/fraction compatibility:
//     magnetic     <-> ferrous        OK
//     eddy_current <-> non_ferrous    OK
//     density      <-> any            OK
//     anything else → DFM-SORT-COMPAT info (suboptimal pairing)
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

namespace separator_sortation {

constexpr const char* kSkillId = "separator_sortation";

struct Input
{
    std::string  method           = "magnetic";   // magnetic | density | eddy_current
    std::string  target_fraction  = "ferrous";    // ferrous | non_ferrous | polymer | glass
    double       purity_pct       = 95.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace separator_sortation
}  // namespace koocadcam::skill
