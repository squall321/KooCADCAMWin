#pragma once
// @lat: [[engine/skills#schedule_parameter_ramp]]
//
// schedule_parameter_ramp — schedule a linear parameter ramp or sweep
// across N intermediate setpoints over a fixed elapsed time.  Used for
// heat-up profiles, pressure soaks, voltage sweeps, vibration step-stress,
// etc.  No geometric change; only the schedule is recorded.
//
//        value ▲
//              │                                  ╱── end_value
//              │                              ╱
//              │                          ╱           steps = N (intermediate setpoints)
//              │                      ╱               ramp_time_min = total elapsed (min)
//              │                  ╱
//              │              ╱
//   start_value│──────────╱
//              └──────────────────────────────────► time
//                    ramp_time_min
//
// Signature pattern:
//   - pattern.kind            = "schedule_parameter_ramp"
//   - pattern.start_value     = <input>
//   - pattern.end_value       = <input>
//   - pattern.ramp_time_min   = <input>
//   - pattern.steps           = <input>
//   - pattern.step_delta      = (end - start) / steps   (derived)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT          ramp_time_min <= 0          → error
//   DFM-INPUT          steps < 1                   → error
//   DFM-RAMP-SLOPE     start_value == end_value    → info (flat ramp)
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

namespace schedule_parameter_ramp {

constexpr const char* kSkillId = "schedule_parameter_ramp";

struct Input
{
    double  start_value    = 0.0;
    double  end_value      = 100.0;
    double  ramp_time_min  = 60.0;
    int     steps          = 10;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace schedule_parameter_ramp
}  // namespace koocadcam::skill
