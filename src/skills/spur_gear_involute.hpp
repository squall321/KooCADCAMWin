#pragma once
// @lat: [[engine/skills#spur_gear_involute]]
//
// spur_gear_involute — REAL involute-profile spur gear (slice 13).
//
// Cuts z teeth into a cylindrical blank using a mathematically-sampled
// involute flank polyline (24 points per flank) per tooth, replicated z
// times around the gear axis by a multi-cut compound.
//
// Math:
//   r_pitch = m·z/2          (pitch radius)
//   r_base  = r_pitch·cos(α) (base circle)
//   r_tip   = r_pitch + m    (addendum / outer)
//   r_root  = r_pitch − 1.25·m  (dedendum / root)
//
// Involute curve in polar form:
//   x(t) = r_base·(cos(t) + t·sin(t))
//   y(t) = r_base·(sin(t) − t·cos(t))   for t ∈ [0, t_max]
//
// DFM checks:
//   DFM-GEAR-MOD   module_mm in (0.3, 50] mm
//   DFM-GEAR-CT    teeth_count in [8, 200]
//   DFM-GEAR-FW    face_width_mm > 0
//   DFM-GEAR-PA    pressure_angle in [14.5, 30] deg
//   DFM-GEAR-OD    blank_outer_dia >= 2·r_tip

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spur_gear_involute {

constexpr const char* kSkillId = "spur_gear_involute";

struct Input
{
    double  module_mm           = 2.0;
    int     teeth_count         = 20;
    double  face_width_mm       = 10.0;
    double  pressure_angle_deg  = 20.0;
    double  blank_outer_dia_mm  = 0.0;   // 0 → auto = 2·r_tip
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace spur_gear_involute
}  // namespace koocadcam::skill
