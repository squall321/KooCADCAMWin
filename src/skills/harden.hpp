#pragma once
// @lat: [[engine/skills#harden]]
//
// harden — heat the workpiece above its critical (austenitize) temperature,
// hold to dissolve carbides into austenite, then rapidly cool (quench) to
// form martensite.  The macroscopic GEOMETRY is unchanged; the MATERIAL
// changes:
//
//   - hardness rises sharply (50 – 65 HRC for high-carbon / alloy steels)
//   - residual stress rises (quench-induced)
//   - microstructure: martensitic (lath / plate, < 1 μm features)
//   - dimensional stability: NOT yet at service state — should be followed
//     by `temper` to relieve the brittle as-quenched stress
//
//        ┌─────────────┐                       ┌─────────────┐
//        │             │  harden               │  ░░░░░░░░░  │
//        │   stock     │  (T_aust, t, quench)  │  martensite │
//        │   (soft)    │  ─────────────────▶  │   (hard,    │
//        │             │                       │   brittle)  │
//        └─────────────┘                       └─────────────┘
//          shape identical; HRC ↑; σ_residual ↑↑
//
// Signature pattern:
//   - pattern.kind                       = "harden"
//   - pattern.material                   = <input>
//   - pattern.austenitize_temperature_c  = <input>
//   - pattern.hold_time_min              = <input>
//   - pattern.quench_medium              = "water" | "oil" | "polymer" | "air"
//   - pattern.expected_hardness_hrc      (lookup)
//   - pattern.martensite_pct             (lookup)
//   - pattern.geometry_changed           = false
//   - pattern.requires_temper            = true (best-practice flag)
//
// DFM checks (all INFO-level except input validation):
//   DFM-INPUT             austenitize_temperature_c ≤ 0  → error
//   DFM-INPUT             hold_time_min ≤ 0              → error
//   DFM-HT-MATERIAL       material unknown               → info
//   DFM-HT-TEMP-*         temp outside material range    → info
//   DFM-HT-QUENCH         quench-medium incompatibility  → info
//   DFM-HT-MEDIUM         quench_medium not in known set → info
//
// Synthesis:
//   NO geometric change.  Clone the workpiece (shape + material) and stamp
//   the signature.
//
// Recognize:
//   Impossible from B-rep alone (martensite is a sub-OCCT microstructure).
//   Returns ONLY signatures replayed from `workpiece.features()`.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace harden {

constexpr const char* kSkillId = "harden";

struct Input
{
    std::string  material                   = "carbon_steel_1045";
    double       austenitize_temperature_c  = 845.0;
    double       hold_time_min              = 30.0;
    std::string  quench_medium              = "oil";   // "water" | "oil" | "polymer" | "air"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace harden
}  // namespace koocadcam::skill
