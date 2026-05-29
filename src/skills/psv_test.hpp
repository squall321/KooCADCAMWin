#pragma once
// @lat: [[engine/skills#psv_test]]
//
// psv_test — pressure-safety-valve (PSV) bench test.  The PSV is removed
// from the vessel, mounted on a test stand, and slowly pressurised with N₂
// or air until the valve "pops" (lifts).  The set pressure is recorded and
// the blowdown — the pressure drop between popping and reseating — is
// measured.  This is the mandatory post-fabrication and periodic-inspection
// qualification for any pressure-relief device.
//
//                                ▲     pressure
//                  set_pressure ─┼─────●  ◄── valve pops
//                                │     │
//                                │     ▼  blowdown (5 – 10 % of set)
//        reseat_pressure ────────┼─────●  ◄── valve reseats
//                                │
//                                └───────────► time
//
// Signature pattern:
//   - pattern.kind                 = "psv_test"
//   - pattern.set_pressure_kpa
//   - pattern.blowdown_pct
//   - pattern.reseat_pressure_kpa  = set × (1 − blowdown_pct/100)
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   DFM-INPUT             set_pressure_kpa ≤ 0  or  blowdown_pct ≤ 0
//   DFM-PSV-BLOWDOWN      blowdown_pct < 2 → reseat too close to set (chatter
//                                            risk, error)
//   DFM-PSV-BLOWDOWN      blowdown_pct > 15 → poor reseating (error)
//   DFM-PSV-PRESSURE      set_pressure_kpa > 50 000 (≈ 500 bar) → ultra-high
//                                            pressure regime, requires
//                                            certified test stand (info)
//
// Synthesis:
//   No geometric change; stamps a signature documenting the qualification
//   parameters for the certification record.
//
// Recognize:
//   Metadata replay only.  A PSV's set pressure cannot be inferred from its
//   B-rep — it is a calibration property of the device's spring.

#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace psv_test {

constexpr const char* kSkillId = "psv_test";

struct Input
{
    double  set_pressure_kpa = 1000.0;   // ≈ 10 bar typical low-pressure relief
    double  blowdown_pct     = 7.0;      // 5 – 10 % typical for steam PSV
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace psv_test
}  // namespace koocadcam::skill
