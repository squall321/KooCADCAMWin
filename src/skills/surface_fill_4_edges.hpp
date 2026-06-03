#pragma once
// @lat: [[engine/skills#surface_fill_4_edges]]
//
// surface_fill_4_edges — Fill a region bounded by exactly 4 polylines with
// a curved Face using BRepFill_Filling at G0 (positional) continuity.
//
//      ─── edge0 ───
//     │             │
//   edge3         edge1
//     │             │
//      ─── edge2 ───
//
// Algorithm:
//   1. Convert each input polyline (vector<gp_Pnt>) to a TopoDS_Edge by
//      chaining BRepBuilderAPI_MakeEdge segments into a TopoDS_Wire and
//      taking the wire's first edge OR each segment edge — we add every
//      segment to BRepFill_Filling as a G0 constraint.
//   2. Run BRepFill_Filling.Build().
//   3. Attach the resulting Face to the workpiece as a compound child.
//
// Signature pattern:
//   pattern.kind             = "surface_fill_4_edges"
//   pattern.is_compound      = true
//   pattern.source_face_count= 0     (no source face — built from edges)
//   pattern.derived_area     = area of the filled face
//   pattern.derived_perimeter= total length of input polylines
//
// DFM:
//   DFM-EDGE-COUNT   : exactly 4 boundary polylines required.
//   DFM-EDGE-LENGTH  : each polyline must have ≥ 2 points.

#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <array>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace surface_fill_4_edges {

constexpr const char* kSkillId = "surface_fill_4_edges";

struct Input
{
    // Each boundary edge is a polyline of ≥ 2 points.
    // Exactly 4 boundaries required.
    std::array<std::vector<gp_Pnt>, 4>  boundary_edges;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace surface_fill_4_edges
}  // namespace koocadcam::skill
