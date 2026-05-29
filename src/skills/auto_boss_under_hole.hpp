#pragma once
// @lat: [[engine/skills#auto_boss_under_hole]]
//
// auto_boss_under_hole — CONTEXT-AWARE compound macro that ensures a
// fastener boss exists directly under a target XY position.  The skill
// inspects the workpiece DOWNWARD from `position_xy`:
//
//   - If solid material is encountered within `kFloatTol_mm` mm of the
//     start point, no fill pillar is added — only the counterbore + pilot
//     pair is cut from the existing solid.
//   - If the topmost solid encountered along the downward ray is FARTHER
//     than `kFloatTol_mm`, a SYNTHESIZED cylindrical pillar (boss) is
//     fused from the start Z down to the nearest solid (or to a sane
//     default if no solid is found, using the workpiece bbox bottom).
//
// Sub-features (subfeature_count = 3 in floating mode, 2 in solid mode):
//   1.   [conditional] cylindrical fill pillar (boss)        [fuse]
//   2.   counterbore seat at top                              [cut]
//   3.   pilot hole through the bottom                        [cut]
//
// Lookup table — Socket-head cap screw seat dimensions per ISO 4762:
//   M3   → seat dia 6.0, seat depth 3.4, pilot dia 3.4
//   M4   → seat dia 8.0, seat depth 4.4, pilot dia 4.5
//   M5   → seat dia 10.0, seat depth 5.4, pilot dia 5.5
//   M6   → seat dia 11.0, seat depth 6.4, pilot dia 6.6
//   M8   → seat dia 15.0, seat depth 8.4, pilot dia 9.0
//
// DFM:
//   DFM-INPUT       : position_xy missing / boss_dia <= 0.
//   DFM-BOSS-SIZE   : unknown screw_spec (must be in lookup table).
//   DFM-BOSS-WALL   : boss_dia must be >= seat_dia + 2 mm (wall ≥ 1 mm).
//   DFM-BOSS-DEPTH  : pillar fill height ≤ 50 mm (mouldability rule).
//
// Recognize:
//   - Geometric: cylinder face + concentric larger cylinder + planar top
//     above the bbox top → boss recognized.
//   - Fallback: metadata replay from wp.features().

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace auto_boss_under_hole {

constexpr const char* kSkillId = "auto_boss_under_hole";

struct Input
{
    FaceDatum   entry_face;                          // top reference face (for datum)
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    double      start_z_mm      = 0.0;               // walk-down start Z (top of design envelope)
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };  // walk direction (into material)
    double      boss_dia_mm     = 8.0;               // outer dia of synthesized pillar
    std::string screw_spec      = "M3";              // M3/M4/M5/M6/M8
    double      total_depth_mm  = 8.0;               // pilot extends this far past seat
};

SkillOutput   apply   (const Workpiece& wp, const Input& in);
DFMReport     validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace auto_boss_under_hole
}  // namespace koocadcam::skill
