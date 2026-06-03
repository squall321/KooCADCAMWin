#pragma once
// @lat: [[engine/skills#bevel_gear]]
//
// bevel_gear — REAL involute-profile bevel (conical) gear (slice 13).
//
// Builds a conical blank (frustum) of the given cone half-angle, then cuts
// z involute tooth-gap profiles around the cone axis.  Each tooth-gap
// cutter is a tapered prism — the profile shrinks linearly from the outer
// (large) end to the inner (small) end of the cone.
//
// DFM checks:
//   DFM-GEAR-MOD    module_mm > 0.3
//   DFM-GEAR-CT     teeth_count in [8, 200]
//   DFM-GEAR-FW     face_width > 0
//   DFM-GEAR-PA     pressure_angle in [14.5, 30]
//   DFM-GEAR-CONE   cone_angle_deg in [10, 80]

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bevel_gear {

constexpr const char* kSkillId = "bevel_gear";

struct Input
{
    double  module_mm           = 2.0;
    int     teeth_count         = 20;
    double  face_width_mm       = 10.0;
    double  cone_angle_deg      = 45.0;   // half-angle of the bevel cone
    double  pressure_angle_deg  = 20.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bevel_gear
}  // namespace koocadcam::skill
