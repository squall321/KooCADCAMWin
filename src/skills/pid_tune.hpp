#pragma once
// @lat: [[engine/skills#pid_tune]]
//
// pid_tune — PID-loop tuning record for a process-control feedback channel
// (temperature loop, pressure loop, flow loop, etc.).  The macroscopic
// workpiece GEOMETRY is unchanged; this skill ONLY stamps the controller
// gain set + target response onto the workpiece feature history so
// downstream process planning / commissioning can verify the loop tune.
//
//                    setpoint  ────►(+)────► [PID]  ────► plant ─┬──► measured
//                                      ▲                          │
//                                      └──────────────────────────┘
//                                            feedback
//
//   kp                proportional gain
//   ki                integral gain
//   kd                derivative gain
//   target_response_ms  desired settling time (ms)
//   overshoot_pct       allowable overshoot (%)
//
// Signature pattern:
//   - pattern.kind                 = "pid_tune"
//   - pattern.kp, ki, kd           = recorded gains
//   - pattern.target_response_ms   = <input>
//   - pattern.overshoot_pct        = <input>
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   DFM-INPUT       kp < 0, ki < 0, kd < 0           → error
//   DFM-INPUT       target_response_ms ≤ 0           → error
//   DFM-PID-OVER    overshoot_pct outside [0, 50]    → error
//   DFM-PID-TUNE    kp == 0 (open loop)              → info
//
// Synthesis:
//   NO geometric change — clone shape + material verbatim, copy features,
//   append signature.
//
// Recognize:
//   Metadata-only (controller gains live below B-rep).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pid_tune {

constexpr const char* kSkillId = "pid_tune";

struct Input
{
    double      kp                  = 1.0;
    double      ki                  = 0.1;
    double      kd                  = 0.01;
    double      target_response_ms  = 250.0;
    double      overshoot_pct       = 5.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pid_tune
}  // namespace koocadcam::skill
