#pragma once
// @lat: [[engine/skills#quench]]
//
// quench — stand-alone rapid cool from an elevated temperature.  This is
// NOT the quench step of the harden chain (which is bundled inside
// `harden::apply` for steels).  Instead, `quench` covers the standalone
// process variants where rapid cooling is the operation in its own right:
//
//   - Solution quench for precipitation-hardening alloys (Al-6061, Al-7075,
//     Ti-6Al-4V) — freeze the high-temperature solid-solution phase so the
//     subsequent aging cycle (out of scope) can precipitate strengthening
//     phases.
//   - Solution-anneal quench for austenitic stainless (316) — retain a
//     single-phase austenite, restore corrosion resistance.
//   - Bulk-quench utility for any work performed at temperature (e.g.
//     after a bend / forming op done hot, to lock in the deformed shape).
//
//        ┌─────────────┐                       ┌─────────────┐
//        │ ▒▒▒▒▒▒▒▒▒▒▒ │  quench               │ ░░░░░░░░░░░ │
//        │   solid     │  (T_solution, t)      │ supersatd.  │
//        │   solution  │  ─────────────────▶  │ solid soln  │
//        │   (hot)     │                       │             │
//        └─────────────┘                       └─────────────┘
//          shape identical;  solid_solution_pct ≈ high
//
// Signature pattern:
//   - pattern.kind                  = "quench"
//   - pattern.material              = <input>
//   - pattern.from_temperature_c    = <input>
//   - pattern.quench_medium         = <input>
//   - pattern.quench_time_s         = <input>
//   - pattern.solid_solution_pct    (estimate: 95 if medium adequate, 70 if not)
//   - pattern.geometry_changed      = false
//
// DFM checks (info-level except DFM-INPUT):
//   DFM-INPUT             from_temperature_c ≤ 0 / quench_time_s ≤ 0  → error
//   DFM-HT-MATERIAL       material unknown                            → info
//   DFM-HT-TEMP-*         from_temperature outside solution range     → info
//   DFM-HT-QUENCH         medium/material incompatibility             → info
//   DFM-HT-MEDIUM         medium not in known set                     → info
//   DFM-HT-SLOW           quench_time_s > 60 s (likely not "rapid")   → info
//
// Synthesis:
//   NO geometric change.
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace quench {

constexpr const char* kSkillId = "quench";

struct Input
{
    std::string  material            = "aluminum_6061";
    double       from_temperature_c  = 530.0;       // typical 6061 solution temp
    std::string  quench_medium       = "water";     // "water" | "oil" | "polymer" | "air"
    double       quench_time_s       = 5.0;         // time to room temp
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace quench
}  // namespace koocadcam::skill
