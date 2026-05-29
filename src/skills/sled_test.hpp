#pragma once
// @lat: [[engine/skills#sled_test]]
//
// sled_test — destructive acceleration-pulse (HALT-style) test record.
// A part is mounted on a sled and subjected to a haversine / trapezoidal
// pulse (peak G, duration ms) emulating crash-grade transient loading
// (automotive crash, telecom shock, satellite launch).  CAD geometry
// does not change; the skill captures the pulse parameters as a
// signature so reliability engineering can audit / replay.
//
//          G_peak
//            ▲         ╱╲
//            │        ╱  ╲       ← pulse_g_peak
//            │       ╱    ╲
//            │      ╱      ╲
//            │     ╱        ╲
//            └────┴──────────┴──────► time
//                  ◀ duration_ms ▶
//
// Signature pattern:
//   - pattern.kind             = "sled_test"
//   - pattern.pulse_g_peak
//   - pattern.duration_ms
//   - pattern.impact_class     ("crash"|"shock"|"launch"|"generic")
//   - pattern.delta_v_m_s      (haversine ≈ π/2 · g · pulse · 1e-3 / g)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   pulse_g_peak  > 0                       (DFM-INPUT error)
//   pulse_g_peak  ≤ 1000                    (DFM-SLED-G info)
//   duration_ms   > 0                       (DFM-INPUT error)
//   duration_ms   ∈ [1, 500]                (DFM-SLED-DURATION info)
//   impact_class  ∈ known set               (DFM-SLED-CLASS info)

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sled_test {

constexpr const char* kSkillId = "sled_test";

struct Input
{
    double      pulse_g_peak = 30.0;          // g
    double      duration_ms  = 30.0;          // ms (haversine half-width)
    std::string impact_class = "crash";       // "crash|shock|launch|generic"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sled_test
}  // namespace koocadcam::skill
