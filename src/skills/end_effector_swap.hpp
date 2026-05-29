#pragma once
// @lat: [[engine/skills#end_effector_swap]]
//
// end_effector_swap — metadata-only record of swapping the robot's
// end-of-arm tooling.  Captures the prior EOAT id, the new EOAT id,
// and the TCP calibration offset.  No geometric change to the workpiece.
//
// Signature pattern:
//   - pattern.kind                 = "end_effector_swap"
//   - pattern.old_effector_id      = <input>
//   - pattern.new_effector_id      = <input>
//   - pattern.calibration_offset_mm = <input>
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   old_effector_id non-empty               (DFM-INPUT error)
//   new_effector_id non-empty               (DFM-INPUT error)
//   old_effector_id == new_effector_id      (DFM-EOAT-SAME info — no-op swap)
//   |calibration_offset_mm| > 10.0          (DFM-EOAT-OFFSET info — large drift)
//
// Synthesis:
//   NO geometric change.  Clone shape+material verbatim, copy prior
//   features, append swap signature.
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

namespace end_effector_swap {

constexpr const char* kSkillId = "end_effector_swap";

struct Input
{
    std::string old_effector_id        = "gripper_2f";
    std::string new_effector_id        = "vacuum_pad_4x";
    double      calibration_offset_mm  = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace end_effector_swap
}  // namespace koocadcam::skill
