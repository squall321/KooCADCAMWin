#pragma once
// @lat: [[engine/skills#alarm_threshold_set]]
//
// alarm_threshold_set — process-alarm threshold record.  Binds a parameter
// (temperature, pressure, flow, vibration, …) to a low / high tripband and
// declares the action the controller should take when the band is breached.
//
//                              high  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
//                               │   ╱╲                   ╱╲
//                               │  ╱  ╲     normal      ╱  ╲
//             parameter         │ ╱    ╲___band________╱    ╲
//                               │                            ╲
//                              low   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─\─
//                                                              ▼
//                                                      action (alert / shutdown / interlock)
//
// Signature pattern:
//   - pattern.kind             = "alarm_threshold_set"
//   - pattern.alarm_id         = <input>
//   - pattern.parameter_name   = <input>
//   - pattern.low              = <input>
//   - pattern.high             = <input>
//   - pattern.action           = <input>  ("alert" | "shutdown" | "interlock")
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT          alarm_id empty                 → error
//   DFM-INPUT          parameter_name empty           → error
//   DFM-ALARM-BAND     low >= high                    → error
//   DFM-ALARM-ACTION   action not in known set        → info
//
// Synthesis:
//   NO geometric change.
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace alarm_threshold_set {

constexpr const char* kSkillId = "alarm_threshold_set";

struct Input
{
    std::string  alarm_id        = "";
    std::string  parameter_name  = "";
    double       low             = 0.0;
    double       high            = 0.0;
    std::string  action          = "alert";    // "alert" | "shutdown" | "interlock"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace alarm_threshold_set
}  // namespace koocadcam::skill
