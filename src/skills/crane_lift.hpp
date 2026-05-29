#pragma once
// @lat: [[engine/skills#crane_lift]]
//
// crane_lift — overhead (bridge / gantry) crane lifting cycle of a heavy
// part to a programmed lift height under a rated lifting capacity.  This
// is a pure material-handling operation: the workpiece geometry is NOT
// modified; only the lift parameters are recorded so a downstream cell
// controller or operator instruction emitter can issue the corresponding
// lift / lower sequence.
//
//          ╔════════════════════════════════════════════════════════╗
//          ║       ─────[ bridge ]─────                             ║
//          ║              │                                         ║
//          ║              │  hook + hoist                           ║
//          ║              ▼                                         ║
//          ║          ┌───────┐    ←─ capacity_kg                   ║
//          ║          │ part  │                                     ║
//          ║          └───────┘     ↑ lift_height_m                 ║
//          ║          ─── floor ─────                               ║
//          ╚════════════════════════════════════════════════════════╝
//
// Signature pattern:
//   - pattern.kind                = "crane_lift"
//   - pattern.lifting_capacity_kg
//   - pattern.lift_height_m
//   - pattern.lift_class          = "light" | "standard" | "heavy" | "ultra"
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   lifting_capacity_kg > 0                     (DFM-INPUT error)
//   lift_height_m > 0                           (DFM-INPUT error)
//   lifting_capacity_kg ≤ 50000                 (DFM-CRANE-CAPACITY info — typ overhead 50 t)
//   lift_height_m ≤ 25                          (DFM-CRANE-HEIGHT info — typ bay headroom)
//   lift_class ∈ known set                      (DFM-CRANE-CLASS info)
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

namespace crane_lift {

constexpr const char* kSkillId = "crane_lift";

struct Input
{
    double       lifting_capacity_kg = 2000.0;
    double       lift_height_m       = 5.0;
    std::string  lift_class          = "standard";  // "light" | "standard" | "heavy" | "ultra"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace crane_lift
}  // namespace koocadcam::skill
