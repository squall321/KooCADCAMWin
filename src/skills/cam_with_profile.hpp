#pragma once
// @lat: [[engine/skills#cam_with_profile]]
//
// cam_with_profile — a cam disc whose outer profile is defined by a
// polyline of (theta_rad, r_mm) waypoints, extruded axially to form a 3D
// cam, with a central shaft bore CUT through it.
//
// Compound feature (≥ 2 ops):
//   step 1: BUILD a closed wire from the polyline waypoints (each point is
//           (cos(θ)·r, sin(θ)·r) in XY), build a face, EXTRUDE along +Z by
//           thickness_mm. This produces a 3D cam disc.
//   step 2: FUSE the cam disc onto the workpiece.
//   step 3: CUT central shaft bore.
//   step 4: CUT optional keyway slot (rectangular box) on the bore wall
//           when keyway_width_mm > 0.
//
// References:
//   DIN 8606 (cam profile machinery standard)
//   ISO 13399 (cutting tool data for cam profile milling)
//
// DFM gate:
//   - waypoint_count ≥ 8 (lower = polygon, not a cam)
//   - thickness_mm ≥ 2.0 mm
//   - all radii ≥ shaft_dia_mm / 2 + 2.0 (wall ≥ 2 mm everywhere)
//   - shaft_dia_mm ≥ 2.0 mm
//   - max_radius / min_radius ≤ 5  (extreme eccentricity rejected — would
//     produce unmachinable cusps)
//
// Recognition:
//   PRIMARY: scan for an axial (+Z) cylindrical face (= shaft bore) plus
//     a non-cylindrical outer profile face/edges (= cam profile).  Count
//     the outer polyline vertices by edge count of the profile boundary.
//   FALLBACK: metadata.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cam_with_profile {

constexpr const char* kSkillId = "cam_with_profile";

struct Waypoint {
    double theta_rad = 0.0;
    double r_mm      = 0.0;
};

struct Input
{
    std::vector<Waypoint> profile;           // (θ, r) polyline; must wrap [0, 2π)
    double thickness_mm   = 10.0;
    double shaft_dia_mm   = 8.0;
    double keyway_width_mm = 0.0;            // 0 → no keyway
    double keyway_depth_mm = 0.0;
    double center_x_mm    = 0.0;
    double center_y_mm    = 0.0;
    double center_z_mm    = 0.0;             // bottom Z; cam extends +Z by thickness
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cam_with_profile
}  // namespace koocadcam::skill
