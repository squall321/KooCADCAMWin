#pragma once
// @lat: [[engine/skills#yaw_motor_mount_compound]]
//
// yaw_motor_mount_compound — Wind turbine yaw drive motor mounting flange
// with cooling vents.  The yaw motor rotates the entire nacelle around the
// tower axis; its mounting flange has a bolt pattern (4/6/8) on a PCD plus
// N angular vent slots cut radially around the motor centerline for
// air-cooling the gearbox / motor brake.
//
//   sub-feature 1..M: M bolt holes via SEQUENTIAL pr::cut
//   sub-feature M+1..M+N: N vent slots (rectangular through-slots), each
//                         placed by gp_Trsf::SetRotation, SEQUENTIAL cut
//
// DFM standard cited:
//   IEC 61400-1 + DIN 13/AS568 vendor refs.  Yaw motor mounts typically
//   use 4/6/8 bolts on PCD (NEMA-style or planetary gearbox flanges).
//   Vent count usually 4/6 to clear bolt positions.
//
// Signature.pattern:
//   { kind = "yaw_motor_mount_compound",
//     is_compound = true,
//     wind_feature_type = "yaw_motor_mount",
//     mount_bolt_count, vent_count, mount_pcd_mm,
//     bolt_thread_size_key, vent_slot_width_mm, vent_slot_length_mm }

#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace yaw_motor_mount_compound {

constexpr const char* kSkillId = "yaw_motor_mount_compound";

struct Input
{
    int    face_id              = -1;
    double center_x_mm          = 0.0;
    double center_y_mm          = 0.0;
    double mount_pcd_mm         = 200.0;
    int    mount_bolt_count     = 6;     // 4 / 6 / 8 typical
    std::string bolt_thread_size_key = "M16";
    int    vent_count           = 6;     // 4 / 6 typical
    double vent_slot_width_mm   = 8.0;
    double vent_slot_length_mm  = 30.0;
    double vent_slot_depth_mm   = 15.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace yaw_motor_mount_compound
}  // namespace koocadcam::skill
