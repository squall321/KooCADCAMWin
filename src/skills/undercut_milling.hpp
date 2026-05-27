#pragma once
// @lat: [[engine/skills#undercut_milling]]
//
// undercut_milling — material removed BELOW (or behind) a surface that a
// conventional 3-axis CAM tool cannot reach.  Examples: T-cutter / lollipop
// tool slots cut from a side face, or pockets requiring 5-axis access.
//
// Geometrically: a rectangular pocket whose access vector is the inward
// normal of a planar SIDE face, not the +Z (top) normal.  The pocket
// extends inward (into the body, along the face's inward normal), with
// width and height along the face surface.
//
//                          entry_face = side face (normal = -X here)
//                                       │
//          (Z)                           │
//           ▲                            │
//           │     ┌──────────────────────┤   ▲
//           │     │ ▲ pocket_height_mm   │   │
//           │     │ │                    │ ◄─┤  workpiece interior
//           │     │ ▼                    │   │
//           │     │ ◄─pocket_width_mm─►  │   ▼
//           │     │
//           │     │ ◄────pocket_depth_mm─►
//           │     └──────────────────────
//           └─────────────────────────────▶  (X)
//
// Signature pattern:
//   - planar pocket walls + flat bottom
//   - pocket access direction = inward normal of the entry face
//     (i.e. NOT aligned with the workpiece's +Z face) — implies a
//     non-vertical tool path
//
// DFM checks:
//   info  : DFM-005 — undercut feature: requires non-3-axis CAM (T-cutter
//           or 5-axis); 3-axis tool cannot reach this geometry
//   DFM-002: pocket_width_mm ≥ 0.8 mm
//
// Recognize:
//   1. Find pocket-like features whose access direction (from open edge to
//      deepest face) does NOT pass through the workpiece's top face — i.e.
//      the deepest interior face has a normal whose vertical component is
//      small.  Heuristic, lower confidence.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace undercut_milling {

constexpr const char* kSkillId = "undercut_milling";

struct Input
{
    FaceDatum   entry_face;            // a planar SIDE face of the workpiece
    double      position_y_mm = 0.0;   // pocket center along entry face's local Y
    double      position_z_mm = 0.0;   // pocket center along entry face's local Z
    double      pocket_width_mm  = 0.0; // dimension along entry face (Y direction)
    double      pocket_depth_mm  = 0.0; // dimension inward (along face normal)
    double      pocket_height_mm = 0.0; // dimension along entry face (Z direction)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace undercut_milling
}  // namespace koocadcam::skill
