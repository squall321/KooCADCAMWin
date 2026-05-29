#pragma once
// @lat: [[engine/skills#remelt]]
//
// remelt — re-melt sorted scrap to ingot.  Pure-metadata operation: the
// post-shred / post-sortation scrap is melted in a furnace, refined, and
// re-cast as ingot for downstream re-use.  Records melt temperature,
// refining method, and recovered yield (%).
//
//        ┌─────────────┐                       ┌─────────────┐
//        │ sorted frac │   remelt              │   ingot     │
//        │  ▓▓▓▓▓▓▓▓▓  │ ────────────────▶    │ (sig only)  │
//        │             │  (T °C, refining,     │  yield_pct  │
//        │             │   yield %)            │             │
//        └─────────────┘                       └─────────────┘
//
// Signature pattern:
//   - pattern.kind             = "remelt"
//   - pattern.melt_temp_c      = melt-pool temperature (°C)
//   - pattern.refining_method  = "flux" | "vacuum" | "argon_purge" | "none"
//   - pattern.yield_pct        = recovered mass fraction (%)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   melt_temp_c > 0                   (DFM-INPUT error if not)
//   yield_pct ∈ (0, 100]              (DFM-INPUT error if not)
//   refining_method recognized        (DFM-REMELT-REFINE info if not)
//   melt_temp_c < 200                 (DFM-REMELT-TEMP info — likely
//                                      too low for any common metal)
//   melt_temp_c > 1800                (DFM-REMELT-TEMP info — exceeds
//                                      common industrial furnace ceiling)
//   yield_pct < 70                    (DFM-REMELT-YIELD warning — low
//                                      recovery, check dross / oxidation)
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

namespace remelt {

constexpr const char* kSkillId = "remelt";

struct Input
{
    double       melt_temp_c       = 700.0;       // °C
    std::string  refining_method   = "flux";      // flux | vacuum | argon_purge | none
    double       yield_pct         = 90.0;        // recovered mass %
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace remelt
}  // namespace koocadcam::skill
