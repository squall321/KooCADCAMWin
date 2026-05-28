#pragma once
// @lat: [[engine/skills#contour_5ax]]
//
// contour_5ax — 5-axis 3D contour finishing.  The tool follows a 3D
// polyline whose every waypoint carries its OWN local tool tilt, then the
// tool axis is interpolated continuously through the waypoints.  Used for
// finishing complex sculpted surfaces (turbine blade leading/trailing
// edges, impeller pressure faces, mold blends).
//
//   Waypoint i  =  ( x_i, y_i, z_i, tilt_deg_i )
//   - position  =  (x, y, z)
//   - tool axis =  vertical (-Z) rotated by tilt_deg_i around the
//                  polyline-tangent axis at that waypoint
//
//                 P0 ─── P1 ─── P2 ─── P3 ─── P4
//                  \      |      /      \      |
//                   \     |     /        \     |     ← tool axis tilts
//                    \    |    /          \    |       per waypoint
//                  10°   5°   0°         15°  20°
//
// Synthesis (slice-1 approximation):
//   Build one cylinder per waypoint, oriented along the tilted tool axis
//   computed via Rodrigues rotation of -Z around (tangent × -Z).  Fuse all
//   cylinders into one compound and cut from the workpiece in a single
//   Boolean.  Same approximation strategy as swarf_milling but every
//   waypoint can carry an independent tilt — giving a varying-axis chain
//   suitable for turbine-blade-style passes.
//
// DFM checks:
//   DFM-002 : tool_dia_mm ≥ 1.5 mm
//   contour_polyline_3d.size() ≥ 2
//   per-waypoint |tilt_deg| ≤ 60
//   DFM-005 info — "contour_5ax requires 5-axis machine kinematics"
//
// Recognize:
//   Low confidence (~0.3) heuristic — look for a chain of cylindrical
//   faces with VARYING axis directions (different tilt angle from
//   vertical across the chain).  Distinct from swarf_milling, where the
//   tilt is approximately constant.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace contour_5ax {

constexpr const char* kSkillId = "contour_5ax";

struct Input
{
    FaceDatum                              entry_face_datum;
    // Each waypoint: (x_mm, y_mm, z_mm, tilt_deg).  The tool axis at
    // waypoint i is -Z rotated by tilt_deg around (tangent × -Z).
    std::vector<std::array<double, 4>>     contour_polyline_3d;
    double                                 tool_dia_mm    = 0.0;
    double                                 tool_length_mm = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace contour_5ax
}  // namespace koocadcam::skill
