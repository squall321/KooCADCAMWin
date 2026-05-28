#pragma once
// @lat: [[engine/skills#carburize]]
//
// carburize — case-hardening by carbon diffusion.  The workpiece is heated
// in a carbon-rich atmosphere (gas, pack, or liquid) at sub-critical to
// austenitizing temperature (~ 850 – 950 °C) for hours, then quenched.
// Carbon diffuses into the exposed surface, producing a hardened "case"
// 0.1 – 2.0 mm deep over a tough ferritic / pearlitic core.
//
//        ┌─────────────────────┐
//        │ ░░░░░░░░░░░░░░░░░░░ │  ← high-carbon martensite case (HRC 55–65)
//        │   (soft tough core) │  ← unchanged carbon level core
//        │                     │
//        └─────────────────────┘
//          shape identical;  surface HRC ↑↑;  core HRC unchanged
//
// Signature pattern:
//   - pattern.kind                 = "carburize"
//   - pattern.material             = <input>
//   - pattern.temperature_c        = <input>     (typ. 850 – 950 °C)
//   - pattern.time_h               = <input>     (typ. 4 – 24 h)
//   - pattern.case_depth_mm        = <input>     (typ. 0.1 – 2.0 mm)
//   - pattern.target_hardness_hrc  = <input>     (typ. 55 – 65 HRC)
//   - pattern.geometry_changed     = false
//   - pattern.requires_temper      = true        (best-practice flag)
//
// DFM checks:
//   DFM-INPUT                       temperature_c ≤ 0 / time_h ≤ 0  → error
//   DFM-CARBURIZE-CASE              case_depth_mm ∉ [0.05, 3.0]    → error
//   DFM-CARBURIZE-HARDNESS          target_hardness_hrc ∉ [20, 70] → error
//   DFM-CARBURIZE-TEMP              temperature_c ∉ [800, 1000]    → info
//   DFM-CARBURIZE-MATERIAL          non-carburizable family        → info
//
// Synthesis:
//   NO geometric change.  Clone the workpiece (shape + material) and stamp
//   the signature.
//
// Recognize:
//   Impossible from B-rep alone (case is a sub-OCCT diffusion gradient).
//   Returns ONLY signatures replayed from `workpiece.features()`.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace carburize {

constexpr const char* kSkillId = "carburize";

struct Input
{
    std::string  material            = "carbon_steel_1018";
    double       temperature_c       = 925.0;    // typical 850 – 950 °C
    double       time_h              = 8.0;      // typical 4 – 24 h
    double       case_depth_mm       = 1.0;      // typical 0.1 – 2.0 mm
    double       target_hardness_hrc = 60.0;     // typical 55 – 65 HRC
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace carburize
}  // namespace koocadcam::skill
