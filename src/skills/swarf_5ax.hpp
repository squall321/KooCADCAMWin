#pragma once
// @lat: [[engine/skills#swarf_5ax]]
//
// swarf_5ax — TRUE swarf-side milling of a ruled surface defined by paired
// top + bottom guide polylines.  Distinct from the slice-1 `swarf_milling`
// skill (which approximates the cut as a chain of tilted cylinders along a
// SINGLE guide).  Here the caller supplies BOTH the top edge and the bottom
// edge of the ruled surface that the tool's side wall must sweep through,
// so the synthesised cut is geometrically closer to the real ruled cut.
//
//                    top polyline (i=0 .. N-1)
//                       T0───T1───T2───T3
//                        ╲    ╲    ╲    ╲
//                         ╲    ╲    ╲    ╲      ← swept ruled surface
//                          ╲    ╲    ╲    ╲       (the tool side wall
//                           ╲    ╲    ╲    ╲       follows this surface)
//                            B0───B1───B2───B3
//                    bottom polyline (matching count)
//
// Synthesis strategy:
//   The mathematically clean way is BRepOffsetAPI_ThruSections (loft) to
//   build the swept ruled surface, then offset/thicken by tool_radius.
//   That builder is fragile against degenerate guide pairs in OCCT 8.0,
//   so this skill uses a robust BOX-CHAIN APPROXIMATION instead:
//
//     For each i in [0, N-2):
//       - Quad corners = { Ti, Ti+1, Bi+1, Bi }.
//       - Compute local frame whose Z is the ruled "normal" (cross of
//         in-quad edges), X is along (Ti → Ti+1).
//       - Build a thin box of length = |Ti → Ti+1|, width = |Ti → Bi|,
//         thickness = 2 × tool_radius, positioned so its mid-plane
//         sits on the quad → the box bites tool_radius into the part
//         on both sides of the surface.
//     fuse all boxes → compound → ONE Boolean cut.
//
//   This gives a faceted swept solid that overlaps adjacent quads and
//   removes material on either side of the ruled surface, which is what
//   real swarf cuts achieve.  Not machine-ready toolpath, but a faithful
//   geometric model for FeatureSignature + downstream CAPP.
//
// DFM checks:
//   DFM-002 : tool_dia_mm ≥ 1.5 mm
//   guide_polyline_top.size() == guide_polyline_bottom.size() ≥ 2
//   DFM-005 info — "swarf_5ax requires 5-axis machine kinematics"
//
// Recognize (limited):
//   Detection of a ruled surface from final topology alone is hard.  We
//   look for ≥ 2 connected planar faces whose normals are tilted (NOT
//   parallel to any global axis), interpret them as facets of a ruled
//   surface and emit a low-confidence (~0.4) candidate.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace swarf_5ax {

constexpr const char* kSkillId = "swarf_5ax";

struct Input
{
    FaceDatum                              entry_face_datum;
    // Top edge of the ruled surface; world (x,y,z).
    std::vector<std::array<double, 3>>     guide_polyline_top;
    // Bottom edge; must have the same number of points as the top.
    std::vector<std::array<double, 3>>     guide_polyline_bottom;
    double                                 tool_dia_mm = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace swarf_5ax
}  // namespace koocadcam::skill
