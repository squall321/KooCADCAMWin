#pragma once
// @lat: [[engine/skills#vibration_qualify]]
//
// vibration_qualify — vibration test acceptance per MIL-STD-810 / DO-160
// for avionics, mission packages, and structural components.  The test
// excites the part on a shaker over a frequency sweep at a target RMS
// acceleration for a duration.  This skill records the QA acceptance
// (no geometry impact) so the process plan can later certify that the
// release configuration was vibration-qualified.
//
//                ▲ accel (g)
//              g │ ─────────
//                │
//                │           ← swept f ∈ [freq_min, freq_max]
//                └─────────► freq (Hz)
//
// Signature pattern:
//   - pattern.kind            = "vibration_qualify"
//   - pattern.freq_min_hz
//   - pattern.freq_max_hz
//   - pattern.accel_g_rms
//   - pattern.duration_min
//   - pattern.axes_count      = 1 | 3   (single-axis or 3-axis)
//   - pattern.result          = "pass" | "fail"
//   - geometry_changed        = false
//
// DFM checks:
//   freq_min_hz   ≥ 5                     (DFM-VIB-FREQ warning below typical floor)
//   freq_max_hz   ≤ 2000                  (DFM-VIB-FREQ warning above typical ceiling)
//   freq_max_hz   > freq_min_hz           (DFM-INPUT error)
//   accel_g_rms   ∈ (0, 50]               (DFM-VIB-ACCEL error / warning)
//   duration_min  ≥ 1.0                   (DFM-INPUT error if not)
//   axes_count    ∈ {1, 3}                (DFM-VIB-AXES warning if unknown)
//
// Synthesis:
//   NO geometric change.  Clone + stamp signature.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace vibration_qualify {

constexpr const char* kSkillId = "vibration_qualify";

struct Input
{
    double      freq_min_hz   = 20.0;
    double      freq_max_hz   = 2000.0;
    double      accel_g_rms   = 7.7;          // typ. MIL-STD-810 cat. 24 ≈ 7.7 g rms
    double      duration_min  = 60.0;
    int         axes_count    = 3;            // 1 = single-axis, 3 = X+Y+Z
    std::string result        = "pass";       // "pass" | "fail"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace vibration_qualify
}  // namespace koocadcam::skill
