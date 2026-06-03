#pragma once
// @lat: [[engine/skills#circular_pattern]]
//
// circular_pattern — replicate a hole around a rotation axis.
//
// For i ∈ [0, count):
//   theta_i = i * total_angle_deg / count    (radians internally)
//   pos_i   = rot(axis_origin, axis_dir, theta_i) applied to
//             (axis_origin + radial_offset_mm * X̂_⊥)
//
// apply():  for each i build a cylinder along axis_dir and gp_Trsf::SetRotation
//           the cylinder around the axis; bundle into cutMany.
//
// DFM gates:
//   DFM-INPUT  : count ≥ 1, hole_dia_mm > 0, hole_depth_mm > 0,
//                radial_offset_mm > 0, total_angle_deg in (0, 360]
//   DFM-PATT-2 : chord_pitch_mm > hole_dia_mm  (no instance overlap)
//   DFM-002    : hole_dia_mm ≥ 0.8 mm

#include "Skill.hpp"

#include <array>

namespace koocadcam::skill {

class Workpiece;

namespace circular_pattern {

constexpr const char* kSkillId = "circular_pattern";

struct Input
{
    std::array<double,3> axis_origin_xyz  = { 0.0, 0.0, 0.0 };
    std::array<double,3> axis_dir_xyz     = { 0.0, 0.0, 1.0 };
    double               hole_dia_mm      = 0.0;
    double               hole_depth_mm    = 0.0;
    double               radial_offset_mm = 0.0;
    int                  count            = 1;
    double               total_angle_deg  = 360.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace circular_pattern
}  // namespace koocadcam::skill
