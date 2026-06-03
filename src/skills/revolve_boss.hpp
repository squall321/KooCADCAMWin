#pragma once
// @lat: [[engine/skills#revolve_boss]]
//
// revolve_boss — SolidWorks-style revolve add.  Take a 2D profile polyline on
// a half-plane (positive radial side of the axis) and revolve it around an
// axis to produce a solid of revolution, then fuse onto the workpiece.
//
// Inputs:
//   - profile_polyline : closed 2D polyline (vector<pair<double,double>>),
//                        interpreted as (radial, axial) i.e. (x, z) coords
//                        on the XZ half-plane (x ≥ 0)
//   - axis_origin      : gp_Pnt of the axis location in world XYZ
//   - axis_dir         : gp_Dir of the axis direction
//   - revolution_angle_deg : default 360 (full revolve)
//
// apply():  build closed wire from profile_polyline in XZ → MakeFace →
// BRepPrimAPI_MakeRevol around the (origin, dir) axis by revolution_angle →
// prim::fuse onto workpiece.
//
// DFM:
//   - profile vertex_count ≥ 3
//   - all radial coords ≥ 0 (else profile crosses axis)
//   - revolution_angle_deg in (0, 360]
//   - draft / overall solid height (max z − min z of profile) > 0.4 mm
//     (min wall thickness)

#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace revolve_boss {

constexpr const char* kSkillId = "revolve_boss";

struct Input
{
    std::vector<std::pair<double,double>> profile_polyline; // (r, z) on XZ half-plane
    gp_Pnt   axis_origin           { 0.0, 0.0, 0.0 };
    gp_Dir   axis_dir              { 0.0, 0.0, 1.0 };
    double   revolution_angle_deg  = 360.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace revolve_boss
}  // namespace koocadcam::skill
