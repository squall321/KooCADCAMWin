#pragma once
// @lat: [[engine/skills#robot_pick_place]]
//
// robot_pick_place — single-cycle robotic transfer of a part from a
// SOURCE pose to a TARGET pose using a programmable gripper.  This is a
// pure material-handling operation: the workpiece geometry is NOT
// modified; only the world-frame pose is conceptually relocated, and
// the operation is recorded so a downstream cell-controller emitter can
// generate the corresponding robot motion program.
//
//          ╔════════════════════════════════════════════════════════╗
//          ║                                                        ║
//          ║   ┌───────┐         ╭────────╮         ┌───────┐       ║
//          ║   │ part  │ ◀─pick─ │ robot  │ ─place▶ │ part  │       ║
//          ║   │ @ A   │         │ + grip │         │ @ B   │       ║
//          ║   └───────┘         ╰────────╯         └───────┘       ║
//          ║   source_xyz                          target_xyz       ║
//          ║                                                        ║
//          ╚════════════════════════════════════════════════════════╝
//
// Signature pattern:
//   - pattern.kind          = "robot_pick_place"
//   - pattern.source_xyz    = [x, y, z]   (mm, cell frame)
//   - pattern.target_xyz    = [x, y, z]   (mm, cell frame)
//   - pattern.gripper_type  = "parallel_jaw" | "vacuum" | "magnetic" | "soft"
//   - pattern.payload_kg
//   - pattern.travel_mm     = ||target - source||
//   - pattern.geometry_changed = false
//
// DFM checks:
//   payload_kg > 0                              (DFM-INPUT error)
//   payload_kg ≤ 25                             (DFM-ROBOT-PAYLOAD info — typ 6-axis limit)
//   travel_mm ≤ 2500                            (DFM-ROBOT-REACH info — typ envelope)
//   source != target (travel_mm > 0.01)         (DFM-ROBOT-DEGENERATE warning)
//   gripper_type ∈ known set                    (DFM-ROBOT-GRIPPER info)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
//
// Recognize:
//   Metadata-only — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace robot_pick_place {

constexpr const char* kSkillId = "robot_pick_place";

struct Input
{
    std::array<double, 3>  source_xyz   = { 0.0, 0.0, 0.0 };
    std::array<double, 3>  target_xyz   = { 500.0, 0.0, 0.0 };
    std::string            gripper_type = "parallel_jaw";   // "parallel_jaw" | "vacuum" | "magnetic" | "soft"
    double                 payload_kg   = 1.0;
    double                 approach_mm  = 50.0;             // approach/retract offset along +Z
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace robot_pick_place
}  // namespace koocadcam::skill
