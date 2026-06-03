#pragma once
// @lat: [[engine/skills#crown_guard_shoulder_compound]]
//
// crown_guard_shoulder_compound — COMPOUND FEATURE (2 sub-features).
//
// Diving-watch crown shoulder protection.  Differs from the existing
// crown_guard_compound (Panerai-style upper/lower guard pair) in that this
// variant adds a SINGLE radial shoulder boss centred at the crown angular
// position and drills a clearance through-hole through the boss to seat the
// crown.  Inspired by Tudor Pelagos / Seiko diver designs.
//
// Sub-feature chain:
//   1. FUSE a rounded boss (boxed prism with chamfer) onto the case side at
//      guard_angle_deg, radial extent = guard_width × guard_height
//   2. CUT a crown clearance through-hole (Ø = crown_clearance_dia) through
//      the boss + a small distance into the case body to allow crown stem
//      free movement
//
// Output: case body + radial boss with a clean coaxial crown clearance hole.
// Single FeatureSignature, pattern.is_compound = true, pattern.is_watch_feature
// = true, pattern.watch_feature_type = "crown_shoulder_with_clearance".
//
// DFM gates:
//   - guard_thickness_mm ≥ 0.5 mm (sufficient impact wall)
//   - crown_clearance_dia_mm > 1.0 mm (crown stem minimum)
//   - boss_radial_extent (= guard_width / 2 by convention) > crown_clearance_dia / 2 + 0.3
//   - guard_height < 60% of case axial height
//   - clearance > 0.3 mm between boss bottom and case end
//
// Recognize: metadata replay (boss with clearance hole topology — replay).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace crown_guard_shoulder_compound {

constexpr const char* kSkillId = "crown_guard_shoulder_compound";

struct Input
{
    FaceDatum   case_side_face;
    gp_Ax1      case_axis              { gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1) };
    double      case_outer_radius_mm = 0.0;
    double      guard_angle_deg      = 90.0;   // 3 o'clock side default
    double      guard_height_mm      = 4.0;    // axial extent of boss
    double      guard_width_mm       = 6.0;    // tangential extent
    double      guard_thickness_mm   = 2.0;    // radial extent (outward from OD)
    double      crown_clearance_dia_mm = 2.5;
    double      crown_z_mm           = 0.0;    // Z of crown axis (boss center)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace crown_guard_shoulder_compound
}  // namespace koocadcam::skill
