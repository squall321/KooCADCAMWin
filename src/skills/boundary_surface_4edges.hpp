#pragma once
// @lat: [[engine/skills#boundary_surface_4edges]]
//
// boundary_surface_4edges — SolidWorks-style "Boundary Surface" with 4
// boundary curves. Builds a free-form (BSpline) face using BRepFill_Filling
// constrained by the 4 boundary polyline edges, then fuses the resulting
// face/shell onto the workpiece (or returns it as a compound when fuse
// is not possible — surface, not solid).
//
// Signature pattern:
//   pattern.kind                = "boundary_surface_4edges"
//   pattern.is_compound         = true
//   pattern.section_count       = 4    (4 boundary curves)
//   pattern.path_length_mm      (sum of 4 boundary lengths)
//   pattern.derived_volume_mm3  (0 — this is a surface, not a solid)
//
// DFM:
//   DFM-INPUT  : exactly 4 boundary curves required.
//   DFM-BSURF-LEN : each curve length must be > 0.

#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <array>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace boundary_surface_4edges {

constexpr const char* kSkillId = "boundary_surface_4edges";

struct Input
{
    // 4 boundary polylines. Endpoints should chain to form a closed loop:
    //   boundaries[0].back() ≈ boundaries[1].front(), etc.
    std::array<std::vector<gp_Pnt>, 4>  boundaries;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace boundary_surface_4edges
}  // namespace koocadcam::skill
