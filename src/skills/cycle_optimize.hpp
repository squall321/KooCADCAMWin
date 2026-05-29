#pragma once
// @lat: [[engine/skills#cycle_optimize]]
//
// cycle_optimize — metadata-only record of a robot cycle-time optimization
// pass.  Captures the prior cycle time, the optimized cycle time, and the
// method used (e.g. "trajectory_blend", "parallel_grip", "skip_air_move").
// No geometric change to the workpiece.
//
// Signature pattern:
//   - pattern.kind                = "cycle_optimize"
//   - pattern.original_cycle_s    = <input>
//   - pattern.optimized_cycle_s   = <input>
//   - pattern.method              = <input>
//   - pattern.savings_pct         = derived ((orig - opt) / orig * 100)
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   original_cycle_s > 0                    (DFM-INPUT error)
//   optimized_cycle_s > 0                   (DFM-INPUT error)
//   optimized_cycle_s > original_cycle_s    (DFM-CYCLE-REGRESSION error — slower)
//   method empty                            (DFM-INPUT warning)
//   savings_pct < 5                         (DFM-CYCLE-MARGINAL info — not worth)
//
// Synthesis:
//   NO geometric change.  Clone shape+material verbatim, copy prior
//   features, append optimization signature.
//
// Recognize:
//   Metadata-only — replay signatures from workpiece.features().

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cycle_optimize {

constexpr const char* kSkillId = "cycle_optimize";

struct Input
{
    double      original_cycle_s   = 30.0;
    double      optimized_cycle_s  = 24.0;
    std::string method             = "trajectory_blend";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cycle_optimize
}  // namespace koocadcam::skill
