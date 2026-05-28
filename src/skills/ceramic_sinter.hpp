#pragma once
// @lat: [[engine/skills#ceramic_sinter]]
//
// ceramic_sinter — fire a green-state ceramic body to densify it.
//
// A green body (powder + binder) is heated to a peak temperature
// (typically 1200–1700 °C for alumina / zirconia / silicon-carbide), held
// to allow grain-boundary diffusion, then slow-cooled.  The part shrinks
// uniformly by `shrinkage_pct` — at the modeling layer we treat the input
// workpiece as the FINAL (post-shrink) shape and record the shrinkage as
// metadata so a fixture / CAM planner can size the green body up by the
// inverse factor.
//
// Signature pattern:
//   pattern.kind            == "ceramic_sinter"
//   pattern.temp_peak_c, pattern.dwell_min, pattern.atmosphere,
//   pattern.shrinkage_pct
//   pattern.geometric_change == false
//
// DFM checks:
//   DFM-SINTER-TEMP   : temp_peak_c ∈ [800, 2000] °C
//   DFM-SINTER-DWELL  : dwell_min ≥ 5
//   DFM-SINTER-ATMOS  : atmosphere ∈ { air, argon, nitrogen, vacuum, hydrogen }
//   DFM-SINTER-SHRINK : shrinkage_pct ∈ [0, 30] (typical 15–20 % for alumina)
//
// Recognize: metadata-only.  Densified vs green body cannot be inferred
// from the analytic B-Rep envelope.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ceramic_sinter {

constexpr const char* kSkillId = "ceramic_sinter";

struct Input
{
    double       temp_peak_c    = 1500.0;
    double       dwell_min      = 120.0;
    std::string  atmosphere     = "air";   // air | argon | nitrogen | vacuum | hydrogen
    double       shrinkage_pct  = 18.0;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ceramic_sinter
}  // namespace koocadcam::skill
