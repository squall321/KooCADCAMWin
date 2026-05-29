#pragma once
// @lat: [[engine/skills#inverter_test]]
//
// inverter_test — record an end-of-line test of an EV traction inverter.
// The inverter itself is unchanged geometrically; this skill captures the
// measured peak power and efficiency so QA can drop / accept / route the
// unit for rework.  Sits at the very end of a "build inverter → flash →
// run-in → test" sequence.
//
//      ┌──── inverter under test ────┐
//      │  IGBT / SiC                  │   peak_power_kw  ← measured
//      │  3-phase output ──→ dyno     │   efficiency_pct ← measured
//      └──────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "inverter_test"
//   - pattern.inverter_id      = <input>
//   - pattern.peak_power_kw    = <input>
//   - pattern.efficiency_pct   = <input>
//   - pattern.test_result      = "PASS" | "FAIL"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   inverter_id non-empty                       (DFM-INPUT error)
//   peak_power_kw > 0                           (DFM-INPUT error)
//   efficiency_pct ∈ (0, 100]                   (DFM-INV-EFF error)
//   efficiency_pct ≥ 90 → PASS  else            (DFM-INV-EFF info FAIL)
//
// Synthesis: NO geometric change. Clone + stamp.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace inverter_test {

constexpr const char* kSkillId = "inverter_test";

struct Input
{
    std::string  inverter_id    = "INV-EV-001";
    double       peak_power_kw  = 200.0;   // measured peak shaft power equiv.
    double       efficiency_pct = 96.5;    // η at rated load
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace inverter_test
}  // namespace koocadcam::skill
