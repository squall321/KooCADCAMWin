#pragma once
// @lat: [[engine/skills#revolve_cut]]
//
// revolve_cut — SolidWorks-style revolve cut.  Dual of revolve_boss: build a
// solid of revolution from a 2D profile and SUBTRACT it from the workpiece.
//
// Identical inputs to revolve_boss; only the final Boolean changes from
// fuse → cut.
//
// DFM identical, plus: the resulting cut must not exceed the stock bbox
// (max radial radius ≤ outerRadiusXY + 2 mm sanity bound).

#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace revolve_cut {

constexpr const char* kSkillId = "revolve_cut";

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

}  // namespace revolve_cut
}  // namespace koocadcam::skill
