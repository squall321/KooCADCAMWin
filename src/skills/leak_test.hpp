#pragma once
// @lat: [[engine/skills#leak_test]]
//
// leak_test — pressure, vacuum, or helium mass-spectrometer leak test for
// pressurised aerospace assemblies (fuel tanks, hydraulic manifolds,
// cabin pressure vessels).  The test does not change the workpiece; it
// stamps a QA record asserting the leak rate is below the acceptance
// threshold.  Helium leak tests are the gold standard for crewed cabins
// and propellant systems.
//
//        ┌─────────────┐     ↘ leak rate (Pa·m³/s)
//        │             │      ↘
//        │  pressurised│       ── < max_leak_rate_pa_m3_s? PASS
//        │  workpiece  │
//        │             │
//        └─────────────┘
//
// Signature pattern:
//   - pattern.kind                  = "leak_test"
//   - pattern.test_type             = "pressure" | "vacuum" | "helium"
//   - pattern.test_pressure_pa
//   - pattern.max_leak_rate_pa_m3_s
//   - pattern.duration_s
//   - pattern.result                = "pass" | "fail"
//   - geometry_changed              = false
//
// DFM checks:
//   test_type ∈ {pressure, vacuum, helium}       (DFM-LEAK-TYPE warning if unknown)
//   test_pressure_pa        > 0                  (DFM-INPUT error if not)
//   max_leak_rate_pa_m3_s   > 0                  (DFM-INPUT error if not)
//   duration_s              ≥ 30                 (DFM-LEAK-DUR info if too short)
//   helium tests with leak rate > 1e-5 Pa·m³/s   (DFM-LEAK-HE info — typical He floor)
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

namespace leak_test {

constexpr const char* kSkillId = "leak_test";

struct Input
{
    std::string test_type             = "pressure";   // "pressure" | "vacuum" | "helium"
    double      test_pressure_pa      = 1.0e5;        // gauge pressure at which we test
    double      max_leak_rate_pa_m3_s = 1.0e-6;       // acceptance threshold
    double      duration_s            = 60.0;         // hold time
    std::string result                = "pass";       // "pass" | "fail"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace leak_test
}  // namespace koocadcam::skill
