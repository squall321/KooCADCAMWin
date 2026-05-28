#pragma once
// @lat: [[engine/skills#seam_weld]]
//
// seam_weld — a continuous welded line joining two parts along a path.
// Geometrically: a thin STADIUM-shaped bead fused on top of a planar face.
//
//   Top view (entry_face):
//                              ╭───────────────────╮
//          start (x,y) ─────── │═══════════════════│ ───── end (x,y)
//                              ╰───────────────────╯
//                              ↑                 ↑
//                              ╰───── bead_width_mm
//
//   Side view (perpendicular to start→end direction):
//                              _____________________
//                            ╱                       ╲   ↕ bead_height_mm
//   ───────────────────────╱_________________________╲────────  ← entry_face
//
// Footprint: stadium = cylinder + box + cylinder (mill_slot inverse — but
// ADDITIVE rather than SUBTRACTIVE).
//
// Signature pattern:
//   - 2 half-cylinder bead-end faces
//   - 2 long planar walls parallel to the start→end direction
//   - 1 top planar face (stadium-shaped)
//   - bead protrudes outward from entry_face by bead_height_mm
//
// DFM checks:
//   bead_width  ∈ [3, 12]
//   bead_height ≤ bead_width × 0.4
//   |end - start| > bead_width  (otherwise it is a spot weld)
//
// Recognize:
//   Pairs of half-cylinder faces with matching radius (both axes
//   perpendicular to a common TOP planar bead face), connected by 2 long
//   planar walls.  Same topology as mill_slot but the stadium is ABOVE the
//   parent face, not BELOW.  We distinguish by looking at whether the bead
//   top face is BIGGER or SMALLER than the workpiece face it sits on.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace seam_weld {

constexpr const char* kSkillId = "seam_weld";

struct Input
{
    FaceDatum   entry_face;
    double      start_x_mm     = 0.0;
    double      start_y_mm     = 0.0;
    double      end_x_mm       = 0.0;
    double      end_y_mm       = 0.0;
    double      bead_width_mm  = 0.0;  // typ. 3 – 8 mm
    double      bead_height_mm = 0.0;  // typ. 0.5 – 2 mm
    std::string material       = "carbon_steel";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace seam_weld
}  // namespace koocadcam::skill
