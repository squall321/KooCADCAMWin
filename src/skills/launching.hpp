#pragma once
// @lat: [[engine/skills#launching]]
//
// launching — record of a vessel's final transfer from the building dock /
// slipway into the water.  Three classical methods are captured:
//
//   side      — vessel parallel to shore, tipped sideways off a slipway
//               (large-river yards, Great Lakes, China yangtze)
//   end       — vessel stern-first down a longitudinal slipway
//               (classic UK / Tyne pattern)
//   floating  — graving dock flooded; vessel floats off blocks
//               (most modern megayards — Hyundai, Samsung, DSME)
//
// Macroscopic B-rep is unchanged; we record the launch event metadata.
//
//        ┌────────┐                  ┌────────┐
//        │ vessel │  side launch     │ vessel │   end launch
//        └────────┘  ───────────▶    └────────┘   ────stern────▶
//        ════════════════              ═══╲═════════════
//          slipway                     slipway (sloped)
//
//        ┌──────────────┐
//        │   vessel     │  floating launch (graving dock flood)
//        └──────────────┘
//        ▓▓▓ flooded ▓▓▓
//
// Signature pattern:
//   - pattern.kind             = "launching"
//   - pattern.launch_method    = "side" | "end" | "floating"
//   - pattern.vessel_mass_t    = <input>
//   - pattern.geometry_changed = false
//
// DFM checks:
//   launch_method  ∈ {side, end, floating}        (DFM-LAUNCH-METHOD error
//                                                  if outside)
//   vessel_mass_t  > 0                            (DFM-INPUT error)
//   vessel_mass_t  ≤ 5000 for side                (DFM-LAUNCH-SIDE info — side
//                                                  launching impractical for
//                                                  very large vessels)
//
// Synthesis: clone shape + material, stamp signature.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace launching {

constexpr const char* kSkillId = "launching";

struct Input
{
    std::string launch_method = "floating";   // "side" | "end" | "floating"
    double      vessel_mass_t = 50000.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace launching
}  // namespace koocadcam::skill
