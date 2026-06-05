#pragma once
// @lat: [[engine/skills#blade_root_t_bolt_array]]
//
// blade_root_t_bolt_array — Wind turbine blade root T-bolt insert pattern.
// Utility-scale wind turbine blades attach to the rotor hub via a ring of
// T-bolt inserts embedded in the blade root flange.  Each T-bolt has:
//   1. an axial through-bore for the bolt shank (M27/M30/M36)
//   2. a counterbore recess at the entry face for the T-bolt head
//
// This skill places N T-bolt holes evenly around a bolt pitch-circle on an
// annular root flange via SEQUENTIAL pr::cut (no compound cutter — each
// rotation is its own cut against the running workpiece).  This is the
// canonical "drill_bolt_circle_with_counterbore" wind-energy primitive.
//
//   sub-feature count = 2 × bolt_count
//     (one through-bore + one counterbore per T-bolt position)
//
// DFM standard cited:
//   IEC 61400-2 / GL "Guidelines for the Certification of Wind Turbines"
//   Utility-scale blade root flanges: 60–120 bolts per blade, M27/M30/M36
//   shank diameter, T-bolt head recess 1.6 × shank diameter typical.
//
// Signature.pattern:
//   { kind = "blade_root_t_bolt_array",
//     is_compound = true,
//     wind_feature_type = "blade_root_t_bolt_array",
//     bolt_count, t_bolt_dia_mm, recess_dia_mm, bolt_pcd_mm }

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace blade_root_t_bolt_array {

constexpr const char* kSkillId = "blade_root_t_bolt_array";

struct Input
{
    int    face_id              = -1;    // annular root flange (planar top)
    double center_x_mm          = 0.0;   // PCD center (XY)
    double center_y_mm          = 0.0;
    double bolt_pcd_mm          = 2400.0;
    int    bolt_count           = 80;    // utility-scale: 60–120
    double t_bolt_dia_mm        = 30.0;  // M27 / M30 / M36 shank
    double t_bolt_depth_mm      = 200.0; // through-bore depth
    double recess_dia_mm        = 50.0;  // counterbore for T-bolt head
    double recess_depth_mm      = 25.0;  // counterbore depth
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace blade_root_t_bolt_array
}  // namespace koocadcam::skill
