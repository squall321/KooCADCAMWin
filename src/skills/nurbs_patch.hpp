#pragma once
// @lat: [[engine/skills#nurbs_patch]]
//
// nurbs_patch — a free-form NURBS surface patch built from a 4×4 grid of
// control points, materialised as a thin shell face and fused onto the
// workpiece as an additive feature.
//
//          v ─►
//        ┌──────────────────────────┐
//        │ • P00 • P01 • P02 • P03  │
//        │ • P10 • P11 • P12 • P13  │  u
//        │ • P20 • P21 • P22 • P23  │  │
//        │ • P30 • P31 • P32 • P33  │  ▼
//        └──────────────────────────┘
//
// Slice-1 simplification: a B-spline surface is constructed from the
// nu × nv control-point grid via GeomAPI_PointsToBSplineSurface, the result
// is turned into a single TopoDS_Face via BRepBuilderAPI_MakeFace, and the
// face is fused onto the workpiece.  When the patch outline is closed in
// both directions the skill attempts BRepBuilderAPI_MakeSolid; otherwise
// the additive shape() is the face alone.
//
// Signature pattern:
//   pattern.kind          = "nurbs_patch"
//   pattern.nu, pattern.nv = control-grid dimensions
//   pattern.degree_u/v
//
// DFM:
//   DFM-NURBS-DEG  : effective surface degree ≥ 2 (else patch is a plane
//                    fragment — caller should use a planar primitive).
//   DFM-INPUT      : control grid must be nu × nv with nu, nv ≥ 4.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace nurbs_patch {

constexpr const char* kSkillId = "nurbs_patch";

struct Input
{
    // Row-major control-point grid: control_grid[u][v].  Slice-1 expects
    // a regular 4×4 grid (any nu × nv with nu, nv ≥ 4 also works) — the
    // outer rows/columns become the patch's four corners.
    std::vector<std::vector<gp_Pnt>>  control_grid;
    int                               degree_u   = 3;     // typical bi-cubic
    int                               degree_v   = 3;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace nurbs_patch
}  // namespace koocadcam::skill
