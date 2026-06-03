#pragma once
// @lat: [[engine/skills#sweep_cut_along_curve]]
//
// sweep_cut_along_curve — SolidWorks-style "Swept Cut" feature.
// Sweep a CLOSED 2D profile along a 3D path polyline and SUBTRACT the
// resulting tubular solid from the workpiece (Boolean cut).
//
// Signature pattern:
//   pattern.kind                = "sweep_cut_along_curve"
//   pattern.is_compound         = true
//   pattern.section_count       = 1
//   pattern.path_length_mm
//   pattern.derived_volume_mm3  (≈ profile area × path length, negative
//                                 stock change interpreted as removed)
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

namespace sweep_cut_along_curve {

constexpr const char* kSkillId = "sweep_cut_along_curve";

struct Input
{
    std::vector<std::pair<double,double>>  profile_polyline;
    std::vector<gp_Pnt>                    path_polyline_xyz;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sweep_cut_along_curve
}  // namespace koocadcam::skill
