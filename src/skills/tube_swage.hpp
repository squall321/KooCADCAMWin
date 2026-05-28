#pragma once
// @lat: [[engine/skills#tube_swage]]
//
// tube_swage — reduce the outer diameter of one end of a tube/cylindrical
// stock by rebuilding that axial range as a smaller-OD cylinder.
//
//                ┌────────────────────┐  R_initial (= bbox.outerRadiusXY())
//                │                    │
//                │                    │
//                │       ┌────────────┘     ← shoulder (normal +Z at start_z)
//                │       │          R_target = target_od_mm / 2
//                │       │
//                └───────┘                  ← end_z (tube end)
//
// The swage section spans [start_z, end_z].  The synthesis cuts the existing
// material in that axial range with an oversized cylindrical cutter (down to
// the workpiece axis), then fuses back a new cylinder of radius R_target.
// This is the classic "rebuild" strategy — it is geometrically simpler than
// a partial radial cut and produces a clean B-rep regardless of any internal
// bore (the inner hole, if present, is preserved by re-cutting it back in).
//
// Signature pattern:
//   - 1 cylindrical face (axis = +Z, radius = target_od/2) over the swage zone
//   - 1 annular planar shoulder face (normal ±Z) at start_z
//   - axis along +Z
//
// DFM checks:
//   DFM-INPUT  : target_od_mm > 0, end_z > start_z
//   DFM-TUBE   : target_od_mm < 2 × bbox.outerRadiusXY() (must reduce)
//   DFM-TUBE   : swage Z range must lie inside the stock Z range
//   DFM-SWAGE  : reduction ratio (R_initial - R_target)/R_initial ≤ 0.6
//                (single-pass swage practical limit)
//
// Recognize:
//   Iterate cylindrical faces with axis = +Z whose radius is strictly less
//   than the workpiece bbox outer radius AND whose Z range includes one of
//   the bbox ends (zMin or zMax — swaging is an *end* operation).  Adjacent
//   planar shoulder face (normal ±Z) confirms the match.

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tube_swage {

constexpr const char* kSkillId = "tube_swage";

struct Input
{
    double  start_z_mm    = 0.0;   // axial start of swage zone
    double  end_z_mm      = 0.0;   // axial end of swage zone (> start_z_mm)
    double  target_od_mm  = 0.0;   // outer diameter after swaging
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tube_swage
}  // namespace koocadcam::skill
