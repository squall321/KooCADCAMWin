#pragma once
// @lat: [[engine/skills#jic_flare_port_seat]]
//
// jic_flare_port_seat — COMPOUND MACRO for an SAE J514 / MS33656 JIC 37°
// flare port: a 37° conical seating chamfer (the actual sealing surface
// against the flare nut's seat), a straight UN/UNF thread above the seat
// for retention, and a top sealing chamfer to deburr the entry.
//
//                          ┌── face ──┐
//                          │          │
//   ═════════════════════════════════════════
//          /───────\           ← face chamfer (sub-cut #3)
//         /         \
//        │  thread   │           ← UN/UNF straight thread (sub-cut #2)
//        │ (straight)│
//         \         /
//          \  37°  /             ← 37° flare seat (sub-cut #1)
//           \     /                 — the actual sealing surface
//            \   /
//             \_/                ← thru-bore continuation
//                          │          │
//                          └──────────┘
//
// JIC sizes (SAE J514, AS5202).  Dash-N corresponds to tube OD in 1/16":
//   -04 : tube OD 1/4"  (6.35),  thread = 7/16-20 UNF
//   -06 : tube OD 3/8"  (9.53),  thread = 9/16-18 UNF
//   -08 : tube OD 1/2"  (12.70), thread = 3/4-16 UNF
//   -10 : tube OD 5/8"  (15.88), thread = 7/8-14 UNF
//   -12 : tube OD 3/4"  (19.05), thread = 1 1/16-12 UNF
//   -16 : tube OD 1"    (25.40), thread = 1 5/16-12 UNF
//
// Sub-feature chain (3 primitive cuts):
//   1. 37° flare seat (cone frustum, included angle 74° → 37° per side).
//   2. UN/UNF straight thread region (cylindrical bore at major dia,
//      depth = thread_engagement_mm).
//   3. Top sealing chamfer (small 45° face chamfer to deburr the
//      thread mouth).
//
// DFM gates:
//   DFM-INPUT       : non-positive sizes / unknown jic_size.
//   DFM-JIC-SIZE    : jic_size out of supported list.
//   DFM-JIC-DEPTH   : thread_engagement < 0.5 × thread_dia (won't seal).
//   DFM-JIC-WALL    : part_thickness < seat_depth + thread_engagement.
//
// Recognize: metadata replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace jic_flare_port_seat {

constexpr const char* kSkillId = "jic_flare_port_seat";

struct Input
{
    FaceDatum   face_id;
    double      position_x_mm     = 0.0;
    double      position_y_mm     = 0.0;
    gp_Dir      axis_dir          { 0.0, 0.0, -1.0 };
    std::string jic_size;                                  // "-04" | "-06" | "-08" | "-10" | "-12" | "-16"
    double      thread_engagement_mm = 0.0;                // 0 = auto (1.0 × thread_dia)
    double      chamfer_leg_mm    = 0.5;                   // top deburr chamfer
    double      part_thickness_mm = 25.0;                  // DFM gate
};

// Look up jic_size; returns true on hit with thread_major_dia_mm,
// thread_pitch_mm, seat_small_dia_mm filled in.
bool resolveJicSize(const std::string& size,
                    double& thread_major_dia_mm,
                    double& thread_pitch_mm,
                    double& seat_small_dia_mm);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace jic_flare_port_seat
}  // namespace koocadcam::skill
