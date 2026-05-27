#pragma once
// @lat: [[engine/skills#mill_keyway]]
//
// mill_keyway — rectangular keyway slot: a flat-bottomed slot with sharp
// (square) rectangular ends, the canonical pocket geometry for sliding key
// joints (Woodruff/parallel keys, sled keys, machine-spindle locating keys).
//
//                  entry_face (planar)
//                         │
//        ─────────────────┼──────────────────────
//                         │
//                  ┌──────┴──────────────┐    ↑ depth_mm
//                  │                     │    │   (flat bottom)
//                  └─────────────────────┘    ↓
//                  ↑                     ↑
//               start                   end
//
// Footprint (top view): plain rectangle = width_mm × length(start→end).
//
// This differs from `mill_slot` (stadium footprint with rounded ends): a
// keyway has **sharp** rectangular corners.  In practice, keyways are not
// cut with a finishing end-mill at the bottom of the slot; rather, they are
// either broached, slotted, or pre-existing in the casting.  The skill is
// documented here as the analytical geometry for CAM recognition and for
// CAD model authoring.
//
// Signature pattern:
//   - 4 planar wall faces (the long sides + 2 short end walls)
//   - 0 cylindrical corner-fillet faces (the key differentiator from
//     mill_rect_pocket and mill_slot)
//   - 1 planar flat-bottom face
//
// DFM checks:
//   DFM-002 : width_mm ≥ 0.8 mm  (broach/slotter min)
//   depth/width > 5 → warning (excessive depth ratio)
//
// Recognize:
//   1. Find 4-wall pockets with **no** corner cylindrical fillets.
//   2. The 4 walls must share a common flat bottom face, and their normals
//      must lie in the entry-face plane (i.e. perpendicular to entry normal).
//   3. The walls must be pairwise parallel (2 long, 2 short).
//   4. Recover: start, end = midpoints of the two short walls; width =
//      distance between the long walls; depth = wall height.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mill_keyway {

constexpr const char* kSkillId = "mill_keyway";

struct Input
{
    FaceDatum   entry_face;
    double      start_x_mm = 0.0;
    double      start_y_mm = 0.0;
    double      end_x_mm   = 0.0;
    double      end_y_mm   = 0.0;
    gp_Dir      axis_dir   { 0.0, 0.0, -1.0 };
    double      width_mm   = 0.0;
    double      depth_mm   = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mill_keyway
}  // namespace koocadcam::skill
