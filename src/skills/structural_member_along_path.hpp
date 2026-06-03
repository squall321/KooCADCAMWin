#pragma once
// @lat: [[engine/skills#structural_member_along_path]]
//
// structural_member_along_path — SolidWorks Weldments "Structural Member"
// feature.  Sweep a parametric cross-section profile (square tube, round
// tube, I-beam, angle, square solid bar) along a 3D path polyline using
// BRepOffsetAPI_MakePipe, then FUSE onto the workpiece.
//
//   profile  ─── sweep ───►   path P0 ── P1 ── P2 ── …
//   (square_tube / round_tube / i_beam / angle / square_solid)
//
// Signature pattern:
//   pattern.kind          = "structural_member_along_path"
//   pattern.is_compound   = true
//   pattern.profile_kind
//   pattern.path_length_mm
//   pattern.derived_volume_mm3
//
// DFM:
//   DFM-INPUT      : ≥ 2 path waypoints, profile_dim > 0.
//   DFM-WALL       : profile_dim > 2 × wall_thick (otherwise the inner wall
//                    eats through the outer wall).
//   DFM-PATH       : total path length > profile outer dimension (member
//                    must be longer than its cross-section).
//   DFM-COLLINEAR  : consecutive path waypoints must be distinct.

#include "Skill.hpp"

#include <array>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace structural_member_along_path {

constexpr const char* kSkillId = "structural_member_along_path";

struct Input
{
    // One of: "square_tube" | "round_tube" | "i_beam" | "angle" | "square_solid"
    std::string                          profile_kind   = "square_tube";
    double                               profile_dim_mm = 50.0;  // outer dim
    double                               wall_thick_mm  = 3.0;   // 0 → solid
    std::vector<std::array<double, 3>>   path_polyline_xyz;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace structural_member_along_path
}  // namespace koocadcam::skill
