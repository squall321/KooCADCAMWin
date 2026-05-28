#pragma once
// @lat: [[engine/skills#lance]]
//
// lance — partial slit (no material removed along most of its length) + a
// small bend, leaving a tab raised out of the sheet.  Used for louvres,
// tabs, and vent slats.
//
//                   ┌──────────────────────┐
//                   │                      │
//                   │   ╱─── tab raised    │
//                   │  ╱       above sheet │   ← lift_mm (positive = +Z)
//                   │ ┃                    │
//                   │ ┃  slit line         │
//                   │ ┃ (cut, kerf = 0)    │   ← cut_length_mm
//                   │ ┃                    │
//                   └─┸─────────────────────┘
//
// Signature records BOTH the cut and the bend together in one pattern so
// re-execution doesn't decompose into separate cut + bend skills.  The
// "cut" half is a knife-edge slit (kerf ≈ 0.1 × sheet_thickness) along the
// polyline waypoints; the "bend" half is the tab on one side of the slit,
// lifted by `lift_mm` (or, equivalently, a rotation about the hinge axis
// at one end of the slit).
//
// Geometry strategy (slice-1 approximation):
//   1. Build a thin stadium-chain cutter along `waypoints` with a very
//      narrow kerf (= 0.1 × thickness) extruded to full thickness.  This
//      slits the sheet without removing meaningful material.
//   2. Build a "tab box" — a rectangular region bounded on one side by the
//      slit polyline and on the other side by a line `tab_width_mm` away
//      (perpendicular).  Fuse it back as a raised slab translated upward
//      by `lift_mm`.  The hinge stays at the far end of the slit (the tab
//      is "lifted out of" the slit).
//
// For the slice-1 approximation we use the stadium-chain offset polyline
// (`pr::offsetPolylineCutter`) for both the slit AND the tab footprint,
// then add the tab back as a fused box of height `lift_mm` standing on the
// sheet's top.  This produces a TopoDS_Shape that has the correct
// signature topology (slit + raised tab) for downstream RE.
//
// Signature pattern:
//   - 1 thin vertical-walled slit channel through the sheet
//   - 1 raised tab slab above sheet top (height = lift_mm)
//   - cut_path_length_mm + tab_width_mm + lift_mm
//
// DFM checks:
//   DFM-L-MIN-LIFT  : lift_mm ≥ sheet_thickness
//   DFM-L-MAX-LIFT  : lift_mm ≤ 5 × sheet_thickness
//   DFM-L-MIN-LEN   : cut polyline length ≥ 3 × sheet_thickness
//   DFM-L-MIN-TAB   : tab_width_mm ≥ sheet_thickness
//   DFM-L-SHEET     : sheet thickness ≤ 5 mm
//   DFM-INPUT       : waypoints.size() ≥ 2, no duplicate points
//
// Recognize:
//   Find a planar +Z face whose Z is above the main sheet top by some
//   `lift_mm`, with thin vertical walls connecting back to the main sheet
//   (the tab).  Confidence ~ 0.70.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lance {

constexpr const char* kSkillId = "lance";

struct Input
{
    // 2D XY polyline waypoints — the slit path.  Projected onto the sheet's
    // top face during synthesis.
    std::vector<std::array<double, 2>>  waypoints;

    // Tab width perpendicular to the FIRST segment of the polyline (the
    // tab footprint sits on the +perpendicular side of waypoints[0]→[1]).
    double  tab_width_mm = 0.0;

    // Lift height (positive = +Z).  Equals the cantilevered raise of the
    // free end of the tab.
    double  lift_mm      = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lance
}  // namespace koocadcam::skill
