#pragma once
// @lat: [[engine/skills#tube_flare]]
//
// tube_flare — expand the outer diameter of one end of a tube by rebuilding
// that axial range as a larger-OD cylinder.  This is the geometric inverse of
// tube_swage.  Real flaring uses a conical mandrel forced into the tube end;
// the slice-1 model uses a cylinder of the expanded radius (the conical
// transition is omitted — its precise geometry depends on the mandrel angle
// which is not part of this skill's input).
//
//                ┌────────────┐                       ┌──── R_target
//                │            │                       │     (= target_od/2)
//                │            └───────────────────────┘     ← shoulder
//                │                               R_initial
//                │
//                └───────────────────────────────────────── ← tube end
//
// Signature pattern:
//   - 1 cylindrical face (axis = +Z, radius = target_od/2) over the flare zone
//   - 1 annular planar shoulder face (normal ±Z) at start_z connecting
//     R_initial back to R_target on the flared side
//
// DFM checks:
//   DFM-INPUT  : target_od_mm > 0, end_z > start_z
//   DFM-TUBE   : target_od_mm > 2 × bbox.outerRadiusXY() (must enlarge)
//   DFM-TUBE   : flare Z range overlaps stock Z
//   DFM-FLARE  : expansion ratio (R_target - R_initial)/R_initial ≤ 0.5
//                (single-pass flare practical limit; beyond this the tube
//                wall thins to fracture)
//
// Recognize:
//   Iterate cylindrical faces with axis = +Z whose radius is strictly greater
//   than the workpiece bbox outer radius minus a stage tolerance — i.e. the
//   *largest* radius cylinder.  The flare is at one end of the bbox Z range.

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tube_flare {

constexpr const char* kSkillId = "tube_flare";

struct Input
{
    double  start_z_mm    = 0.0;   // axial start of flare zone
    double  end_z_mm      = 0.0;   // axial end (typically the tube end)
    double  target_od_mm  = 0.0;   // outer diameter after flaring
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tube_flare
}  // namespace koocadcam::skill
