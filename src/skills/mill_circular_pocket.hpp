#pragma once
// @lat: [[engine/skills#mill_circular_pocket]]
//
// mill_circular_pocket — mill a round flat-bottom pocket into a face.
//
// Unlike drill_hole (which uses a twist drill and leaves a conical tip),
// this skill uses an end-mill and leaves a FLAT BOTTOM.  It is also NOT a
// through-feature: it always stops at `depth_mm` below the entry face.
// Optional bottom-corner fillet (`bottom_corner_r_mm`) emulates a corner-
// radius end-mill.
//
//                         entry_face (planar)
//                              │
//                  ────────────┼──────────────
//                              │     ↓ axis
//                              │
//                   ┌──────────┴──────────┐    ↑ depth_mm
//                   │  (flat bottom)      │    │
//                   └─────────────────────┘    ↓
//                              ↑
//                          diameter_mm
//
// Signature pattern:
//   - 1 cylindrical face (wall)
//   - 1 planar face (flat bottom)
//   - 1 circular edge at the entry face
//   - 1 circular edge at the bottom (where wall meets bottom)
//   - If bottom_corner_r_mm > 0: 1 toroidal blend face between wall & bottom
//     (and the bottom circular edge degenerates into the smaller bottom)
//
// DFM checks:
//   DFM-002 : diameter_mm ≥ 0.8 mm  (end-mill min size)
//   DFM-004 : if bottom_corner_r_mm > 0, must be ≥ 0.2 mm
//   depth/dia > 5    → warning (tool chatter / chip evacuation)
//
// Recognize:
//   Pick cylindrical faces adjacent to a SMALL planar face (= flat pocket
//   bottom).  Distinguish from drill_hole by aspect ratio: pockets have
//   depth/dia < 2 typically; if depth/dia > 2 we let drill_hole own it.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mill_circular_pocket {

constexpr const char* kSkillId = "mill_circular_pocket";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm       = 0.0;
    double      position_y_mm       = 0.0;
    gp_Dir      axis_dir            { 0.0, 0.0, -1.0 };
    double      diameter_mm         = 0.0;
    double      depth_mm            = 0.0;
    double      bottom_corner_r_mm  = 0.0;   // 0 = sharp corner
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mill_circular_pocket
}  // namespace koocadcam::skill
