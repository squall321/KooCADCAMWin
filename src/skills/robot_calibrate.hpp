#pragma once
// @lat: [[engine/skills#robot_calibrate]]
//
// robot_calibrate — metadata-only record of a robotic arm calibration
// session.  Captures the qualified accuracy + repeatability achieved on
// a specific physical robot cell.  No geometric change to the workpiece.
//
// Signature pattern:
//   - pattern.kind             = "robot_calibrate"
//   - pattern.robot_id         = <input>
//   - pattern.accuracy_mm      = <input>
//   - pattern.repeatability_mm = <input>
//   - pattern.geometry_changed = false
//
// DFM checks:
//   robot_id non-empty                       (DFM-INPUT error)
//   accuracy_mm > 0                          (DFM-INPUT error)
//   repeatability_mm > 0                     (DFM-INPUT error)
//   repeatability_mm > accuracy_mm           (DFM-ROBOT-RANGE info — unusual)
//   accuracy_mm > 1.0                        (DFM-ROBOT-ACCURACY info — coarse)
//
// Synthesis:
//   NO geometric change.  Clone shape+material verbatim, copy prior
//   features, append calibration signature.
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

namespace robot_calibrate {

constexpr const char* kSkillId = "robot_calibrate";

struct Input
{
    std::string robot_id          = "cell_A_robot_1";
    double      accuracy_mm       = 0.1;
    double      repeatability_mm  = 0.05;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace robot_calibrate
}  // namespace koocadcam::skill
