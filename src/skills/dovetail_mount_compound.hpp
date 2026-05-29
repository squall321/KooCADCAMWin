#pragma once
// @lat: [[engine/skills#dovetail_mount_compound]]
//
// dovetail_mount_compound — COMPOUND MECHANICAL FEATURE: a single dovetail
// mounting receiver with set-screw locking holes and a stop face.
//
// Used for: tool inserts, micro-stage mounts, optical breadboard fingers,
// machinist vise jaws, optics V-block bases.
//
//   ┌─ top of stock (entry_face)
//   ▼
//   ─────────┬────────────────────────────────────────────────┬──────
//            │ dovetail slot 60°                              │
//            │ ╲                                            ╱ │      ↑
//            │  ╲                                          ╱  │      │
//            │   ╲                                        ╱   │      │ slot_depth
//            │    └──────────────────────────────────────┘    │      ↓
//            │                                                 │
//            │  ◯ ←set-screw 1     ◯ ←set-screw 2 (M5×0.8 tapped from side wall)
//            │                                                 │
//            └─────────────────────────────────────────────────┘
//
//   End view (along slot direction):
//
//     ╲    ╱  ← dovetail walls at 60° from horizontal
//      └──┘   ← bottom of dovetail
//
//   The 2 set-screw threaded holes enter from one of the side walls
//   (perpendicular to the dovetail axis), tapped to retain a sliding insert.
//
//   The tee-slot stop is a deeper transverse stop cut at the end of the
//   slot — prevents the dovetail insert from sliding past a limit.
//
// Sub-features (4):
//   1. Dovetail slot (60° walls, slot_length × slot_width × slot_depth).
//   2. Set-screw threaded hole #1 (M5 tapped from side wall, depth = slot_width)
//   3. Set-screw threaded hole #2 (same, at 1/3 and 2/3 along slot length).
//   4. Tee-slot stop (transverse cut at slot end, full depth, 3 mm wide).
//
// Input:
//   - face_id        : entry face (top of stock)
//   - slot_length_mm : along slot axis (X direction default)
//   - slot_width_mm  : top width of dovetail trapezoid
//   - slot_depth_mm  : depth from entry face
//   - dovetail_angle : trapezoid wall angle from vertical (typ. 30° for 60°
//                      included angle); range [15°, 45°]
//
// DFM:
//   DFM-DOVETAIL  dovetail_angle outside [15°, 45°]
//   DFM-DOVETAIL  slot_length < 4 × slot_width (insufficient guide length)
//   DFM-002       slot_width < 2 mm (cutter too small)
//   DFM-INPUT     slot_depth ≤ 0
//
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace dovetail_mount_compound {

constexpr const char* kSkillId = "dovetail_mount_compound";

struct Input
{
    FaceDatum  entry_face;
    double     start_x_mm   = 0.0;   // slot start (center-line on top face)
    double     start_y_mm   = 0.0;
    gp_Dir     slot_dir { 1.0, 0.0, 0.0 };  // along +X by default
    gp_Dir     axis_dir { 0.0, 0.0, -1.0 }; // into the stock

    double     slot_length_mm = 0.0;
    double     slot_width_mm  = 0.0;   // top width
    double     slot_depth_mm  = 0.0;
    double     dovetail_angle_deg = 30.0;  // wall angle from vertical
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace dovetail_mount_compound
}  // namespace koocadcam::skill
