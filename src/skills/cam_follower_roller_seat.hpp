#pragma once
// @lat: [[engine/skills#cam_follower_roller_seat]]
//
// cam_follower_roller_seat — a machined seat for a cylindrical cam-follower
// roller, comprising:
//   1. Cylindrical POCKET for the roller body (radius = roller_dia/2 + clearance)
//   2. Two ARM CLEARANCE slots on either side so the follower arm can swing
//   3. A retention threaded BOSS (modelled as a pilot bore) that takes the
//      axle through the roller centre, perpendicular to the roller seat axis.
//
// Reference standards:
//   ANSI B92.1 (involute splines + rolling-element seat clearance)
//   ISO 286-2 (clearance fit h7 / H8 for rolling elements)
//
// Compound feature (≥ 3 cuts):
//   1. CUT cylindrical pocket — roller body seat, axial direction = +Y
//      (roller axis), depth = roller_length + axial clearance.
//   2. CUT two arm clearance pockets — rectangular pockets perpendicular
//      to the roller axis, allowing the follower arm to swing through.
//   3. CUT retention pilot bore — perpendicular to roller axis (i.e., axial
//      direction = +Z), threaded for the axle bolt.
//
// DFM gate:
//   - roller_dia_mm ≥ 2.0 mm (smallest standard cam follower)
//   - roller_length_mm ≥ 2.0 mm
//   - radial_clearance_mm ∈ [0.05, 0.5] mm  (ISO 286-2 H8)
//   - arm_clearance_width_mm ≥ 0.8 mm
//   - retention_dia_mm ≥ 1.5 mm  (M2 minimum for axle)
//   - retention_dia_mm ≤ roller_dia_mm × 0.6  (axle must fit in roller bore)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cam_follower_roller_seat {

constexpr const char* kSkillId = "cam_follower_roller_seat";

struct Input
{
    double roller_dia_mm           = 12.0;
    double roller_length_mm        = 8.0;
    double radial_clearance_mm     = 0.1;     // ISO 286-2 H8-ish
    double arm_clearance_width_mm  = 4.0;
    double arm_clearance_depth_mm  = 6.0;
    double retention_dia_mm        = 3.3;     // M4 tap drill
    double retention_depth_mm      = 10.0;
    // Seat centre in world frame. The pocket axis is +Y; the retention bore
    // axis is +Z.
    double center_x_mm             = 0.0;
    double center_y_mm             = 0.0;
    double center_z_mm             = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cam_follower_roller_seat
}  // namespace koocadcam::skill
