#pragma once
// @lat: [[engine/skills#mill_rect_pocket]]
//
// mill_rect_pocket — rectangular flat-bottom pocket with rounded vertical
// corners (4 vertical corner fillets of radius `corner_r_mm`).
//
//                    entry_face (planar)
//                          │
//             ─────────────┼─────────────────
//                          │
//                 ┌────────┴────────────┐    ↑ depth_mm
//                 │   ╭─────────╮       │    │
//                 │   │ (flat   │       │    │
//                 │   │  bottom)│       │    │
//                 │   ╰─────────╯       │    │
//                 └─────────────────────┘    ↓
//                       length_mm
//                       (X dir local)
//
// Signature pattern:
//   - 4 planar wall faces (the long sides)
//   - 4 cylindrical corner-fillet faces (the vertical corner blends)
//   - 1 planar bottom face (rounded-rectangle shape)
//   - top: 4 line edges + 4 circular-arc edges at the entry face
//   - bottom: 4 line edges + 4 circular-arc edges at the bottom face
//
// DFM checks:
//   DFM-004 : corner_r_mm ≥ 0.2 mm  (if > 0)
//   DFM-006 : depth_mm ≤ 5.0 mm     (warn if greater; full check needs
//             workpiece-thickness context, deferred to process planner)
//   DFM-001 : min wall thickness    (deferred, info-only)
//
// Recognize:
//   1. Group cylindrical faces by axis direction (must be aligned).
//   2. Find sets of 4 cylinders that share a common bottom planar face.
//   3. The 4 walls are the planar faces adjacent to both bottom + 2 cyls.
//   4. Recover: corner_r = cylinder radius, depth = cyl height, center =
//      mean of the 4 cylinder axis bases, length/width from wall extents.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mill_rect_pocket {

constexpr const char* kSkillId = "mill_rect_pocket";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm   = 0.0;
    double      center_y_mm   = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      length_mm     = 0.0;   // X-aligned extent
    double      width_mm      = 0.0;   // Y-aligned extent
    double      depth_mm      = 0.0;
    double      corner_r_mm   = 1.0;   // vertical-corner fillet
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mill_rect_pocket
}  // namespace koocadcam::skill
