#pragma once
// @lat: [[engine/skills#mill_open_pocket]]
//
// mill_open_pocket — a rectangular pocket OPEN ON ONE SIDE.  The pocket
// "exits" the workpiece on one edge, producing a NOTCH carved from the side
// of the entry face (vs `mill_rect_pocket` which is closed on all 4 sides).
//
//             entry_face (top, planar)
//                       │
//                       ↓
//                       open_direction →
//   ┌─────────────────┬─────────┬─┐
//   │                 │ ╭───────┴─┘  ← notch exits to +X
//   │                 │ │       length_mm INTO workpiece
//   │   workpiece     │ │
//   │                 │ ╰───────┬─┐
//   └─────────────────┴─────────┴─┘
//                          ←──→ width_mm  (along the open edge)
//                          ↓ depth_mm     (vertical into entry face)
//
// Signature pattern (what the skill leaves on the workpiece):
//   - 3 planar wall faces (back + 2 sides, NOT the open side)
//   - 2 cylindrical corner-fillet faces (the 2 corners FARTHEST from the
//     open edge — the back corners; the 2 corners ON the open edge degenerate)
//   - 1 planar bottom face
//   - top: 3 line edges + 2 circular-arc edges (at the entry face)
//
// DFM checks:
//   DFM-004 : corner_r_mm ≥ 0.2 mm  (if > 0)
//   DFM-006 : depth_mm    warning if > 5 mm
//
// Recognize:
//   The signature distinguishes open vs closed pockets by the wall count:
//   3 walls + 2 corner cylinders (open) vs 4 walls + 4 corners (closed).
//   We detect the missing wall by counting corner cylinders sharing a common
//   bottom planar face.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mill_open_pocket {

constexpr const char* kSkillId = "mill_open_pocket";

struct Input
{
    FaceDatum   entry_face;                            // top face the pocket cuts into
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };     // into the entry face
    // Position is the CENTRE OF THE OPEN EDGE (on the entry face).  The
    // pocket extends `length_mm` INTO the workpiece OPPOSITE `open_direction`.
    double      position_x_mm  = 0.0;
    double      position_y_mm  = 0.0;
    // Direction in which the pocket is OPEN (lies in entry face's plane,
    // tangent to the open edge).  Must NOT be axial.  E.g. +X for a notch
    // exiting on the +X side.
    gp_Dir      open_direction { 1.0, 0.0, 0.0 };
    // Length INTO the workpiece (along -open_direction): how deep the
    // notch reaches from the open edge.
    double      length_mm      = 0.0;
    // Width along the open edge.
    double      width_mm       = 0.0;
    // Vertical depth into the entry face.
    double      depth_mm       = 0.0;
    // Back-corner fillet radius (the two corners FARTHEST from the open
    // edge are filleted; the two corners ON the open edge are open).
    double      corner_r_mm    = 1.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mill_open_pocket
}  // namespace koocadcam::skill
