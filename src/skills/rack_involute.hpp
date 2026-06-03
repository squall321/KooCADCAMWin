#pragma once
// @lat: [[engine/skills#rack_involute]]
//
// rack_involute — REAL trapezoidal-flank rack (slice 13).
//
// Builds a rectangular bar (length × face_width × (2·module)) and cuts
// tooth_count trapezoidal gaps along its length at circular pitch =
// π·module.  Each gap has flanks angled at the pressure angle —
// equivalent of an involute flank at infinite pitch radius.
//
// DFM checks:
//   DFM-GEAR-MOD    module_mm > 0.3
//   DFM-GEAR-CT     tooth_count in [2, 500]
//   DFM-GEAR-FW     face_width > 0
//   DFM-GEAR-PA     pressure_angle in [14.5, 30]
//   DFM-GEAR-LEN    rack_length >= tooth_count · π·module

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rack_involute {

constexpr const char* kSkillId = "rack_involute";

struct Input
{
    double  module_mm           = 2.0;
    int     tooth_count         = 10;
    double  face_width_mm       = 10.0;
    double  pressure_angle_deg  = 20.0;
    double  rack_length_mm      = 0.0;   // 0 → auto = tooth_count · π·m + 2·m
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rack_involute
}  // namespace koocadcam::skill
