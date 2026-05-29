#pragma once
// @lat: [[engine/skills#hoist_op]]
//
// hoist_op — chain or wire-rope hoist lifting cycle, typically jib-arm
// or monorail-mounted, used to raise a part to a programmed height under
// the rated hoist capacity.  This is a pure material-handling operation:
// the workpiece geometry is NOT modified; only the hoist parameters are
// recorded so a downstream cell controller or operator instruction
// emitter can issue the corresponding raise / lower sequence.
//
//          ╔════════════════════════════════════════════════════════╗
//          ║                                                        ║
//          ║       ┌── jib / monorail ──┐                           ║
//          ║       │                    │                           ║
//          ║       │  hoist_type:       │                           ║
//          ║       │  chain | wire_rope │                           ║
//          ║       │     │              │                           ║
//          ║       │     ▼ hook         │                           ║
//          ║       │   ┌────┐           │   ↑ lift_height_m         ║
//          ║       │   │part│           │                           ║
//          ║       │   └────┘           │                           ║
//          ║       └────────────────────┘                           ║
//          ╚════════════════════════════════════════════════════════╝
//
// Signature pattern:
//   - pattern.kind              = "hoist_op"
//   - pattern.hoist_type        = "chain" | "wire_rope" | "lever"
//   - pattern.hoist_capacity_kg
//   - pattern.lift_height_m
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   hoist_capacity_kg > 0                       (DFM-INPUT error)
//   lift_height_m > 0                           (DFM-INPUT error)
//   hoist_capacity_kg ≤ 5000                    (DFM-HOIST-CAPACITY info — typ jib hoist)
//   lift_height_m ≤ 10                          (DFM-HOIST-HEIGHT info — typ hoist lift)
//   hoist_type ∈ known set                      (DFM-HOIST-TYPE info)
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

namespace hoist_op {

constexpr const char* kSkillId = "hoist_op";

struct Input
{
    std::string  hoist_type        = "chain";   // "chain" | "wire_rope" | "lever"
    double       hoist_capacity_kg = 500.0;
    double       lift_height_m     = 3.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hoist_op
}  // namespace koocadcam::skill
