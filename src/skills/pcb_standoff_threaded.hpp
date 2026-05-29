#pragma once
// @lat: [[engine/skills#pcb_standoff_threaded]]
//
// pcb_standoff_threaded — COMPOUND ENCLOSURE FEATURE that produces a threaded
// PCB mounting boss.  Three primitive operations fused into one signature:
//
//   1. Boss collar     — raised cylindrical PROTRUSION (fuse, not cut) of
//                        diameter = boss_dia, height = boss_height_mm above
//                        the face, with a top fillet for clearance.
//   2. PCB-clearance recess — a Ø(boss_dia + 1.0 mm) × 0.5 mm shallow rebate
//                        around the boss top for the PCB hole tolerance.
//   3. Tapped blind hole — pilot drill at the ISO 68-1 pilot diameter for
//                        thread_M, sunk into the boss to thread_depth_mm
//                        below the boss top.
//
//                   boss_dia + 1.0
//                ◄────────────────►
//                ┌────────────────┐  ← PCB-clearance recess (#2)
//             ───┴───┐         ┌──┴───   (0.5 mm deep)
//                    │  tap    │       boss_dia
//                    │  hole   │      ◄─────►
//                    │  (#3)   │      boss_height_mm
//             ───────┤  pilot  ├──────
//                    │   dia   │   ← enclosure face
//                    │         │
//                    └─────────┘
//
// DFM checks:
//   DFM-INPUT     : boss_height_mm in [1.5, 25] mm
//   DFM-TAP-SIZE  : thread_M must be one of M1.6/M2/M2.5/M3/M4/M5/M6
//   DFM-002       : pilot dia ≥ 0.8 mm (always true for table values)
//   DFM-BOSS-WALL : boss_dia ≥ pilot_dia + 2.0 mm (minimum 1 mm wall around
//                   the tap — DIN/ISO standard wall ratio)
//   DFM-BOSS-AR   : boss_height / boss_dia ≤ 5 (tall thin bosses warp on
//                   thermal cycling)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pcb_standoff_threaded {

constexpr const char* kSkillId = "pcb_standoff_threaded";

struct Input
{
    FaceDatum   face;
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, 1.0 };  // boss extrudes outward
    std::string thread_M        = "M3";              // ISO metric size
    double      boss_height_mm  = 6.0;
    double      boss_dia_mm     = 0.0;               // 0 → auto = pilot + 4 mm
    double      thread_depth_mm = 0.0;               // 0 → auto = boss_height − 0.5
};

// Resolve thread → pilot diameter (mirrors drill_and_tap's table).  Returns
// 0.0 if size unknown.  Public for tests.
double pilotDiameterFor(const std::string& thread_M);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pcb_standoff_threaded
}  // namespace koocadcam::skill
