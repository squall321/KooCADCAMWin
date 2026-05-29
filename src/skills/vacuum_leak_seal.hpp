#pragma once
// @lat: [[engine/skills#vacuum_leak_seal]]
//
// vacuum_leak_seal — records the leak-sealing action applied to a UHV
// joint that did not pass helium leak-check.  Sealing options include
// re-torque (knife-edge), epoxy fillet (Torr-Seal), atmospheric-side
// braze patch, or full re-make.  Captures the measured leak rate before
// sealing and the resulting ultimate pressure when the chamber is
// re-pumped.  No macroscopic geometry change is modelled; the action is
// metadata-only on the workpiece.
//
//     leak detected ──▶ [ vacuum_leak_seal ] ──▶ re-pump
//                          method
//                          measured leak_rate_pa_m3_s (before)
//                          ultimate_pressure_torr     (after)
//
// Signature pattern:
//   - pattern.kind                  = "vacuum_leak_seal"
//   - pattern.leak_rate_pa_m3_s     = <input>          (before seal)
//   - pattern.seal_method           = <input>
//   - pattern.ultimate_pressure_torr= <input>          (after pump-down)
//   - geometry_changed              = false
//
// DFM checks:
//   leak_rate_pa_m3_s         > 0                          (DFM-INPUT)
//   leak_rate_pa_m3_s         > 1e-7 ⇒ DFM-LEAK-GROSS error (gross leak: re-make joint)
//   seal_method               in known set                 (DFM-LEAK-METHOD info)
//   ultimate_pressure_torr    ∈ (0, 1e-6]                  (DFM-LEAK-UHV info if > 1e-9)
//
// Synthesis: clone + stamp.  Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace vacuum_leak_seal {

constexpr const char* kSkillId = "vacuum_leak_seal";

struct Input
{
    double      leak_rate_pa_m3_s      = 1.0e-9;
    std::string seal_method            = "torr_seal_epoxy"; // "torr_seal_epoxy" | "re_torque" | "braze_patch" | "remake"
    double      ultimate_pressure_torr = 1.0e-9;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace vacuum_leak_seal
}  // namespace koocadcam::skill
