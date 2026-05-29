#pragma once
// @lat: [[engine/skills#ore_flotation]]
//
// ore_flotation — froth-flotation beneficiation.  Finely ground slurry passes
// through a bank of cells; reagents (collectors, frothers, depressants)
// selectively render the target mineral hydrophobic so that rising air
// bubbles carry it to a froth that overflows.  KooCADCAM does not simulate
// reagent chemistry; this skill captures cell count, retention time, and
// expected recovery percent so a process planner can size the bank.
//
//        slurry  →  ▌▌▌▌▌▌ cell_count cells (retention_time_min each) →  concentrate
//                                                                        + tailings
//
// Signature pattern:
//   pattern.kind                = "ore_flotation"
//   pattern.cell_count          = int
//   pattern.retention_time_min  = double
//   pattern.recovery_pct        = double
//   pattern.geometry_changed    = false
//
// DFM:
//   DFM-INPUT             cell_count ≤ 0, retention_time_min ≤ 0, or
//                         recovery_pct outside (0, 100]
//   DFM-RECOVERY-LOW      recovery_pct < 50 (sub-economic, info)
//   DFM-RETENTION-LONG    retention_time_min > 120 (oversized bank, info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ore_flotation {

constexpr const char* kSkillId = "ore_flotation";

struct Input
{
    int    cell_count         = 8;
    double retention_time_min = 12.0;
    double recovery_pct       = 85.0;   // 0–100
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ore_flotation
}  // namespace koocadcam::skill
