#pragma once
// @lat: [[engine/skills#gripper_finger_dovetail_mount]]
//
// gripper_finger_dovetail_mount — Robotic gripper FINGER dovetail interface
// (slice 16, robotics / automation).
//
// The exchangeable finger of a parallel/angular gripper mates to the jaw via
// a dovetail rail.  This skill machines the dovetail SLOT into the finger
// blank (a trapezoidal undercut prism, wider at the bottom than at the mouth)
// plus a transverse clamp-screw bore that pinches the dovetail closed onto
// the rail.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. dovetail slot  (trapezoid prism — wider bottom = the undercut)
//   2. transverse clamp-screw bore (cylinder cut across the slot)
//
// The dovetail is approximated as a single trapezoidal prism (one watertight
// tool), then cut sequentially.  No overlapping-cutter compound boolean.
//
// subfeature_count = 2.
//
// DFM:
//   DFM-INPUT          : every dimension must be > 0.
//   DFM-ROBOTICS-ANGLE : dovetail_angle_deg in (0, 30] (standard 45/55/60-deg
//                        dovetail half-angles map to 30 deg from vertical).
//   DFM-THREAD         : clamp_thread_key must exist in central metric table.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gripper_finger_dovetail_mount {

constexpr const char* kSkillId = "gripper_finger_dovetail_mount";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy         { 0.0, 0.0, 0.0 };
    double      dovetail_width_mm  = 16.0;   // mouth (top) width of the slot
    double      dovetail_angle_deg = 15.0;   // flank angle from vertical
    double      dovetail_depth_mm  = 8.0;
    std::string clamp_thread_key   = "M4";   // from _iso_thread_table.hpp
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gripper_finger_dovetail_mount
}  // namespace koocadcam::skill
