#pragma once
// @lat: [[engine/skills#robot_path_program]]
//
// robot_path_program — programmed multi-waypoint robot motion.  Useful
// when the cell controller needs more than a simple pick-place (e.g.
// glue-dispense path, deburring along an edge, multi-station handoff).
// Each waypoint is an XYZ point in the cell frame; the recorded
// signature carries the ordered list so a downstream G-code / KRL / RAPID
// emitter can render the full path.
//
//        ╭──── waypoint sequence (ordered) ────────────────────────╮
//        │                                                          │
//        │     w0 ─▶ w1 ─▶ w2 ─▶ w3 ─▶ w4 ─▶ … ─▶ wN-1              │
//        │     ●     ●     ●     ●     ●          ●                 │
//        │      \   /│      \   /│            └── end               │
//        │       \ / │       \ / │                                  │
//        │        X  ▼        X  ▼                                  │
//        │       motion_type ∈ {PTP, LIN, CIRC}                     │
//        ╰──────────────────────────────────────────────────────────╯
//
// Waypoint format (in pattern JSON):
//   waypoints = [ [x0, y0, z0], [x1, y1, z1], … ]   (mm, cell frame)
//   N = waypoints.size()                              (≥ 2)
//   total_length_mm = Σ‖wi+1 − wi‖
//
// Signature pattern:
//   - pattern.kind            = "robot_path_program"
//   - pattern.waypoints       = [[x,y,z], …]
//   - pattern.waypoint_count  = N
//   - pattern.total_length_mm
//   - pattern.speed_mm_s
//   - pattern.payload_kg
//   - pattern.motion_type     = "PTP" | "LIN" | "CIRC"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   waypoints.size() ≥ 2                         (DFM-INPUT error)
//   payload_kg > 0                               (DFM-INPUT error)
//   speed_mm_s > 0                               (DFM-INPUT error)
//   total_length_mm ≤ 50_000                     (DFM-ROBOT-PATH info — guard absurd paths)
//   speed_mm_s ≤ 2000                            (DFM-ROBOT-SPEED info — typ envelope)
//   motion_type ∈ {PTP, LIN, CIRC}               (DFM-ROBOT-MOTION info)
//
// Synthesis:
//   NO geometric change.  Compute total path length, clone + stamp.
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

namespace robot_path_program {

constexpr const char* kSkillId = "robot_path_program";

struct Input
{
    std::vector<std::array<double, 3>>  waypoints;             // ordered XYZ in mm
    std::string                         motion_type = "LIN";   // "PTP" | "LIN" | "CIRC"
    double                              speed_mm_s  = 250.0;
    double                              payload_kg  = 1.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace robot_path_program
}  // namespace koocadcam::skill
