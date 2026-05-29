#pragma once
// @lat: [[engine/skills#auto_chamfer_all_outer_edges]]
//
// auto_chamfer_all_outer_edges — CONTEXT-AWARE compound macro that walks
// every edge in the workpiece, classifies each as INNER (concave) or
// OUTER (convex), and chamfers all OUTER convex edges in a single
// BRepFilletAPI_MakeChamfer pass.
//
// Sub-features (subfeature_count = N + 1):
//   1. one chamfer per outer convex edge                       [batched cut]
//   2. one closing weld-relief cut at the highest-curvature
//      junction (so the toolpath has a guaranteed re-entry)    [cut]
//
// Lookup table — ISO 13715 standard chamfer sizes:
//   "small"  → 0.3 mm  (electronic enclosures, fine consumer)
//   "med"    → 0.5 mm  (general mech)
//   "std"    → 1.0 mm  (default for sheet-metal & moulded)
//   "large"  → 2.0 mm  (heavy gauge / safety chamfers)
//
// Context detection:
//   - Use the convexity test: edge midpoint, compare cross-product of the
//     two adjacent face normals.  A convex outer edge has cross direction
//     pointing OUTWARD relative to the workpiece centroid.
//
// DFM:
//   DFM-INPUT       : chamfer_size ≤ 0.
//   DFM-CHAMFER-SPEC: unknown size key.
//   DFM-CHAMFER-MIN : chamfer_size < 0.1 mm (knife-edge).
//   DFM-CHAMFER-MAX : chamfer_size > 0.25 × min wall thickness.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace auto_chamfer_all_outer_edges {

constexpr const char* kSkillId = "auto_chamfer_all_outer_edges";

struct Input
{
    double      chamfer_size_mm     = 1.0;
    std::string chamfer_size_key    = "std";    // small/med/std/large
    bool        include_top_edges   = true;
    bool        include_bottom_edges = true;
    double      min_wall_thick_mm   = 4.0;       // for chamfer-size DFM
};

SkillOutput   apply   (const Workpiece& wp, const Input& in);
DFMReport     validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace auto_chamfer_all_outer_edges
}  // namespace koocadcam::skill
