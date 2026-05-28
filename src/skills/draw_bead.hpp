#pragma once
// @lat: [[engine/skills#draw_bead]]
//
// draw_bead — closed CURVED groove in a sheet, used as a "drawbead" during
// deep-drawing operations to restrain material flow.  A draw_bead is
// always CLOSED (last waypoint equals first), and its cross-section is
// a U-shaped groove cut into the sheet top.
//
//                            ┌────────────┐
//                          ┌─┘ closed     └─┐
//                          │ polyline       │
//                          └─┐ loop       ┌─┘
//                            └────────────┘
//
//             groove cross-section:
//                  ╲  ╱
//                ───╲╱───    ↓ depth_mm
//                  sheet
//
// Geometry strategy:
//   1. Validate that waypoints form a closed loop (first == last, or we
//      auto-close by appending the first).
//   2. Build a stadium-chain cutter along the closed polyline with
//      width = `width_mm` and depth = `depth_mm`.
//   3. Cut from the sheet.
//
// Signature records:
//   - is_closed = true
//   - polyline waypoints, width_mm, depth_mm, path_length_mm
//
// DFM checks:
//   DFM-DB-MIN-W    : width_mm ≥ 3 × sheet_thickness
//   DFM-DB-MIN-D    : depth_mm ≥ sheet_thickness × 0.5
//   DFM-DB-MAX-D    : depth_mm ≤ sheet_thickness × 1.5
//   DFM-DB-CLOSED   : waypoints must form a closed loop (≥ 3 points,
//                     first ≈ last within tolerance)
//   DFM-DB-SHEET    : sheet thickness ≤ 5 mm
//   DFM-INPUT       : waypoints.size() ≥ 3
//
// Recognize:
//   Detect a closed loop of vertical-walled faces at the sheet top whose
//   axial extent equals `depth_mm`.  Approximate via centroid distance
//   path length.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace draw_bead {

constexpr const char* kSkillId = "draw_bead";

struct Input
{
    std::vector<std::array<double, 2>>  waypoints;   // closed loop
    double  width_mm = 0.0;
    double  depth_mm = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace draw_bead
}  // namespace koocadcam::skill
