#pragma once
// @lat: [[engine/skills#bore_with_shelf]]
//
// bore_with_shelf — STEPPED cylindrical bore: narrow upper opening, wider
// lower cavity.  The interior shelf where the two diameters meet is an
// annular planar face.  This is the "watch interior" pattern — the upper
// (smaller) bore is the display window, the lower (larger) bore is the
// movement compartment.
//
//                          ┌─ entry_face (planar)
//                          ▼
//                    ──────┬──────┬─────────────
//                          │ upr  │             ↑ upper_depth_mm
//                          │ dia  │             │
//                          │ ◄►   │             ↓
//                  ────────┴──┬───┴──────────── (annular shelf)
//                             │
//                             │  lower_dia_mm   ↑ lower_depth_mm
//                             │  ◄────────►    │
//                             │                │
//                             └────────────────┘ (blind bottom)
//
// Wait — the *upper* diameter is meant to be the small one (the visible
// window) and the *lower* diameter the wide one (the buried movement
// pocket).  The ASCII above flips them.  See the input spec below — the
// caller passes upper_dia_mm < lower_dia_mm.
//
// Signature pattern (what the skill leaves on the workpiece):
//   - 2 coaxial cylindrical faces with DIFFERENT radii
//   - 1 annular planar face (the "shelf" between them)
//   - 1 bottom planar face (the lower bore's blind bottom)
//   - 3 circular edges (top opening, shelf inner/outer, bottom)
//
// DFM checks:
//   DFM-002 : both diameters ≥ 0.8 mm
//   custom  : lower_dia_mm > upper_dia_mm  (otherwise it's a simple bore)
//   custom  : (upper_depth + lower_depth) > 20 mm → warning
//
// Recognize:
//   Iterate pairs of coaxial cylindrical faces with different radii.  When
//   their bounding circles confirm a shared "shelf" plane between them and
//   a blind bottom under the wider cylinder, recover:
//     upper_dia    = 2 × smaller cyl radius (the entry cylinder)
//     upper_depth  = entry-Z − shelf-Z      (from entry face down to shelf)
//     lower_dia    = 2 × larger cyl radius
//     lower_depth  = shelf-Z − bottom-Z

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bore_with_shelf {

constexpr const char* kSkillId = "bore_with_shelf";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm  = 0.0;
    double      position_y_mm  = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    double      upper_dia_mm   = 0.0;   // narrow opening
    double      upper_depth_mm = 0.0;   // measured from entry face
    double      lower_dia_mm   = 0.0;   // wider lower cavity (> upper_dia_mm)
    double      lower_depth_mm = 0.0;   // measured from shelf (not from entry)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bore_with_shelf
}  // namespace koocadcam::skill
