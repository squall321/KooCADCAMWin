#pragma once
// @lat: [[engine/skills#cam_follower_threaded_seat]]
//
// cam_follower_threaded_seat — COMPOUND MACRO for a stud-type cam-follower seat.
// Combines:
//   1. A tapped blind hole sized for the cam-follower stud thread (M8/M10/M12).
//   2. A counterbore (seat) for the lock nut at the top.
//   3. A chamfer at the entry to guide stud insertion.
//
//                     ╲     /        ← entry chamfer
//                    ┌─╲   /─┐
//                    │  ╲ /  │       ← lock-nut counterbore (larger)
//                    │   ▼   │
//                  ──┘       └──
//                    │ tap   │       ← tapped pilot (M-cam-follower)
//                    │ blind │
//                    │       │
//                    └───────┘
//
// Sub-feature count = 1 tap pilot + 1 counterbore + 1 chamfer = 3.
//
// Standards reference: McMaster / Misumi cam-follower stud-mount specs.
// Lock-nut counterbore: nut OD + 1 mm; lock-nut height table (M8=6.5 mm,
// M10=8 mm, M12=10 mm).  Entry chamfer 45° × 0.5 mm.
//
// DFM checks:
//   DFM-INPUT       : tap_depth, position sane
//   DFM-002         : pilot dia (from M-table) >= 0.8 mm (auto-pass)
//   DFM-CF-TAP      : cam_follower_M in supported set (M6..M16)
//   DFM-CF-DEPTH    : tap_depth >= 1.5 × M-diameter (engagement)
//
// Recognize: metadata-only (compound).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cam_follower_threaded_seat {

constexpr const char* kSkillId = "cam_follower_threaded_seat";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm       = 0.0;
    double      position_y_mm       = 0.0;
    gp_Dir      axis_dir            { 0.0, 0.0, -1.0 };
    std::string cam_follower_M;             // "M8", "M10", "M12", etc.
    double      tap_depth_mm        = 0.0;  // 0 → derived (1.5 × M)
    // Defaults (per M-table):
    //   pilot_dia: from drill_and_tap table (M8=6.8, M10=8.5, M12=10.3)
    //   counterbore_dia: lock-nut OD + 1 mm  (M8=14, M10=18, M12=20)
    //   counterbore_depth: lock-nut height   (M8=6.5, M10=8, M12=10)
    //   chamfer_size: 0.5 mm @ 45°
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cam_follower_threaded_seat
}  // namespace koocadcam::skill
