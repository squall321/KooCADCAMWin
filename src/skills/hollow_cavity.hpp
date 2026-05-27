#pragma once
// @lat: [[engine/skills#hollow_cavity]]
//
// hollow_cavity — bulk-remove interior material to convert a solid into a
// shell.  The cavity inherits the workpiece's outer XY silhouette, inset
// by `wall_thickness_mm`.  Used to make a watch / phone case body from a
// solid block in one step before per-feature finishing.
//
//                   ┌─────────────────────────────────┐
//                   │ wall_thickness                  │
//                   │  ──►┌───────────────────────┐ ◄─│
//                   │     │                       │   │
//                   │     │     cavity (cut)      │   │ ↑ depth_mm
//                   │     │                       │   │ │
//                   │     └───────────────────────┘   │ ↓
//                   └─────────────────────────────────┘
//
// Slice-1 implementation:
//   - Cuboid stock → rectangular cavity (workpiece bbox shrunk by 2 × wall_thickness in XY).
//   - Cylindrical stock → cylindrical cavity (diameter = stock_dia − 2 × wall_thickness).
//   Detection is "best planar face area / largest face is round" — see the
//   isCylindricalStock helper in the .cpp.
//
// Signature pattern:
//   - Cuboid case:  4 planar walls + 1 flat bottom + 1 top opening loop.
//   - Cylindrical:  1 cylindrical wall + 1 flat bottom + 1 top opening loop.
//   - In both cases the cavity face area is large compared to per-feature
//     planar faces; `recognize()` uses that as the heuristic.
//
// DFM checks:
//   DFM-001 : wall_thickness_mm ≥ 0.4 mm (0.35 for watch material — informed
//             by workpiece material when available; here use a single fixed
//             0.4 threshold and warn under 0.4).
//   custom  : depth > 0 (and not exceeding the workpiece's Z span).
//
// Recognize:
//   Heuristic — find the largest planar face that is "inside" the
//   workpiece (i.e. its outward normal points into +axis_dir from caller's
//   POV) and is fully surrounded by perpendicular planar walls.  Recover
//   wall_thickness as (outer bbox − cavity bbox) / 2 in XY; depth as the
//   axial extent between the cavity top opening and the cavity bottom.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hollow_cavity {

constexpr const char* kSkillId = "hollow_cavity";

struct Input
{
    FaceDatum   entry_face;                 // the opening face (where material is removed from)
    double      wall_thickness_mm = 0.0;
    double      depth_mm          = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hollow_cavity
}  // namespace koocadcam::skill
