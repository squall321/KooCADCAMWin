#pragma once
// @lat: [[engine/skills#sweep_surface]]
//
// sweep_surface — sweep a circular profile (a "tube") along a polyline
// path, using BRepOffsetAPI_MakePipe.  The result is a tubular swept
// solid that is fused onto the workpiece — typically used for cooling
// channels in molds, cable conduits in housings, or organic ribbing.
//
//             ◯ ─── ◯ ─── ◯ ─── ◯
//             │           ↑
//             │           waypoint
//             ▼
//          circular profile (radius_mm)
//
// Signature pattern:
//   pattern.kind             = "sweep_surface"
//   pattern.path_waypoint_count
//   pattern.profile_radius_mm
//
// DFM:
//   DFM-INPUT        : ≥ 2 waypoints required.
//   DFM-SWEEP-R      : profile radius_mm must be > 0.
//   DFM-SWEEP-COLL   : consecutive waypoints must be distinct (no
//                      coincident points that would create a degenerate
//                      path edge).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sweep_surface {

constexpr const char* kSkillId = "sweep_surface";

struct Input
{
    std::vector<gp_Pnt>   path_waypoints;        // ≥ 2 points
    double                profile_radius_mm = 1.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sweep_surface
}  // namespace koocadcam::skill
