#pragma once
// @lat: [[engine/skills#cnc_load_unload]]
//
// cnc_load_unload — robotic / gantry transfer of a part into or out of a
// CNC machine work zone.  This is a pure material-handling operation: the
// workpiece geometry is NOT modified; only the world-frame pose changes
// (stock to chuck/vise, or finished part to outfeed conveyor) and the
// operation is recorded so a downstream cell-controller emitter can issue
// the corresponding load/unload program.
//
//          ╔════════════════════════════════════════════════════════╗
//          ║                                                        ║
//          ║   ┌───────┐         ╭────────╮         ┌────────┐      ║
//          ║   │ stock │ ◀─load─ │ robot/ │ ─load─▶ │  CNC   │      ║
//          ║   │ pallet│         │ gantry │         │ chuck  │      ║
//          ║   └───────┘         ╰────────╯         └────────┘      ║
//          ║                                                        ║
//          ╚════════════════════════════════════════════════════════╝
//
// Signature pattern:
//   - pattern.kind          = "cnc_load_unload"
//   - pattern.load_method   = "robot" | "gantry" | "manual"
//   - pattern.payload_kg
//   - pattern.cycle_time_s
//   - pattern.geometry_changed = false
//
// DFM checks:
//   payload_kg > 0                              (DFM-INPUT error)
//   payload_kg ≤ 50                             (DFM-CNC-PAYLOAD info — typ gantry limit)
//   cycle_time_s > 0                            (DFM-INPUT error)
//   cycle_time_s ≤ 60                           (DFM-CNC-CYCLE info — typ load cycle)
//   load_method ∈ known set                     (DFM-CNC-METHOD info)
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

namespace cnc_load_unload {

constexpr const char* kSkillId = "cnc_load_unload";

struct Input
{
    std::string  load_method   = "robot";   // "robot" | "gantry" | "manual"
    double       payload_kg    = 5.0;
    double       cycle_time_s  = 8.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cnc_load_unload
}  // namespace koocadcam::skill
