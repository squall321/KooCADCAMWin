#pragma once
// @lat: [[engine/skills#fiber_splice]]
//
// fiber_splice — fusion splice between two fiber ends, producing a
// permanent low-loss joint.  Cleaved fiber ends are aligned (active or
// passive), then heated until the silica softens and fuses.  Two heating
// methods are common:
//   - arc: tungsten-electrode arc, ~ 2000 °C (most common, field splice)
//   - laser: CO2 laser, more controllable taper but lab-grade
//
//       ┌─────────────┐   gap   ┌─────────────┐
//       │ fiber A   ◇ │  →←     │ ◇   fiber B │
//       └─────────────┘         └─────────────┘
//                  │           │
//                  └─ arc/laser ─┘
//                       ↓
//              ┌─────────────────┐
//              │ fused joint     │  loss_db: 0.01 – 1.0
//              └─────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "fiber_splice"
//   - pattern.loss_db          = <input>
//   - pattern.splice_method    = "arc" | "laser"
//   - pattern.protected        = bool (heat-shrink sleeve recommended)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT              loss_db < 0                              (error)
//   DFM-SPLICE-LOSS-HIGH   loss_db > 1.0                            (error)
//   DFM-SPLICE-LOSS-INFO   loss_db > 0.3                            (info)
//   DFM-SPLICE-METHOD      splice_method ∉ {arc, laser}             (info)
//
// Synthesis:
//   Records splice metadata.  Geometry unchanged — splice is a sub-OCCT
//   joint quality property.
//
// Recognize:
//   Impossible from B-rep; metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace fiber_splice {

constexpr const char* kSkillId = "fiber_splice";

struct Input
{
    double       loss_db        = 0.05;     // typical arc-fusion average
    std::string  splice_method  = "arc";    // "arc" | "laser"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fiber_splice
}  // namespace koocadcam::skill
