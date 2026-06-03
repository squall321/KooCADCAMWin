#pragma once
// @lat: [[engine/skills#sweep_boss_along_curve]]
//
// sweep_boss_along_curve — SolidWorks-style "Swept Boss/Base" feature.
// Sweep a CLOSED 2D profile (polygon) along a 3D path polyline and FUSE
// the resulting tubular solid onto the workpiece.
//
//     profile (XY, normal to path start)        path polyline (3D)
//     ┌────────┐                                 P0 ── P1 ── P2 ── P3
//     │        │     ───sweep───►                       \
//     └────────┘                                         volume added
//
// Signature pattern:
//   pattern.kind                = "sweep_boss_along_curve"
//   pattern.is_compound         = true
//   pattern.section_count       = 1
//   pattern.path_length_mm
//   pattern.derived_volume_mm3  (≈ profile area × path length)
//
// DFM:
//   DFM-INPUT      : ≥ 3 vertices in the profile, ≥ 2 path waypoints.
//   DFM-SWEEP-PATH : path length must be > 0.
//   DFM-SWEEP-PROF : profile must close (≥ 3 distinct vertices).

#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sweep_boss_along_curve {

constexpr const char* kSkillId = "sweep_boss_along_curve";

struct Input
{
    // Closed profile in the XY plane (Z = 0), positioned at path[0] and
    // oriented normal to the first path segment.
    std::vector<std::pair<double,double>>  profile_polyline;
    // 3D path polyline (≥ 2 distinct waypoints).
    std::vector<gp_Pnt>                    path_polyline_xyz;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sweep_boss_along_curve
}  // namespace koocadcam::skill
