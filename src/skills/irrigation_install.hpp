#pragma once
// @lat: [[engine/skills#irrigation_install]]
//
// irrigation_install — installation record for an agricultural irrigation
// system across a target area.  Captures system type (drip/sprinkler/
// flood), covered area in hectares, and per-emitter flow in litres/hour.
// No geometric change to the workpiece.
//
//          ╔════════════════════════════════════════════════════════╗
//          ║                                                        ║
//          ║   ─── drip:       │ │ │ │ │   ← low-flow emitters       ║
//          ║                   ▼ ▼ ▼ ▼ ▼                            ║
//          ║   ─── sprinkler:  ╲│╱   ╲│╱   ← rotating heads          ║
//          ║                   ◯     ◯                              ║
//          ║   ─── flood:      ▓▓▓▓▓▓▓▓▓   ← surface flood             ║
//          ║                                                        ║
//          ║   area_ha × flow_l_h drives pump sizing                ║
//          ║                                                        ║
//          ╚════════════════════════════════════════════════════════╝
//
// Signature pattern:
//   - pattern.kind             = "irrigation_install"
//   - pattern.system_type      = "drip" | "sprinkler" | "flood"
//   - pattern.area_ha
//   - pattern.flow_l_h         = per-emitter / per-head flow
//   - pattern.geometry_changed = false
//
// DFM checks:
//   system_type     ∈ {drip, sprinkler, flood}         (DFM-IRRIG-TYPE error otherwise)
//   area_ha         > 0                                (DFM-INPUT error)
//   flow_l_h        > 0                                (DFM-INPUT error)
//   flood + flow_l_h > 0 typical bound (flood doesn't use l_h per head)
//                                                      (DFM-IRRIG-FLOOD info)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
//
// Recognize:
//   Metadata-only — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace irrigation_install {

constexpr const char* kSkillId = "irrigation_install";

struct Input
{
    std::string  system_type = "drip";       // "drip" | "sprinkler" | "flood"
    double       area_ha     = 5.0;
    double       flow_l_h    = 2.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace irrigation_install
}  // namespace koocadcam::skill
