#pragma once
// @lat: [[engine/skills#temper]]
//
// temper — reheat a previously hardened workpiece to a moderate temperature
// (150 – 700 °C, depending on target hardness) and hold, then cool.
// Tempering trades a portion of the as-quenched hardness for an order-of-
// magnitude gain in ductility / toughness; the workpiece becomes
// dimensionally stable and ready for service.
//
//        ┌─────────────┐                       ┌─────────────┐
//        │ ░░░░░░░░░░░ │  temper               │ ▓▓▓▓▓▓▓▓▓▓▓ │
//        │   hard,     │  (T < critical, t)    │   hard but  │
//        │   brittle   │  ─────────────────▶  │   tough     │
//        │             │                       │             │
//        └─────────────┘                       └─────────────┘
//          shape identical; HRC ↓ slightly;  toughness ↑↑
//
// Signature pattern:
//   - pattern.kind             = "temper"
//   - pattern.material         = <input>
//   - pattern.temperature_c    = <input>
//   - pattern.hold_time_min    = <input>
//   - pattern.post_temper_hrc  (estimated from temperature curve)
//   - pattern.toughness        = "improved"
//   - pattern.geometry_changed = false
//
// DFM checks (mostly info; only DFM-INPUT is an error):
//   DFM-INPUT             temperature_c ≤ 0 / hold_time_min ≤ 0 → error
//   DFM-HT-TEMPER-RANGE   temperature_c ∉ [150, 700] °C         → info
//   DFM-HT-MATERIAL       material not in lookup table          → info
//   DFM-HT-NOT-TEMPERABLE material has HRC_max = 0 (no temper)  → info
//
// Synthesis:
//   NO geometric change.  Clone the workpiece (shape + material) and stamp
//   the signature.
//
// Recognize:
//   Metadata-only.  Tempering does not show on B-rep.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace temper {

constexpr const char* kSkillId = "temper";

struct Input
{
    std::string  material        = "carbon_steel_1045";
    double       temperature_c   = 200.0;   // typical low-temper for tool steel
    double       hold_time_min   = 60.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace temper
}  // namespace koocadcam::skill
