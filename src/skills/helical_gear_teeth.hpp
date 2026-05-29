#pragma once
// @lat: [[engine/skills#helical_gear_teeth]]
//
// helical_gear_teeth — REAL compound feature:
//
//   sub-cuts (N + 1):
//     1. central through bore
//     2..N+1. N helical tooth-flank cutouts at helix_angle, equispaced
//             around the pitch circle.
//
// Tooth count: N = round(pitch_dia / normal_module).
//
// In slice-1 approximation each helical gullet is built as an OBLIQUE
// prism: the gullet face cross-section (5-point involute polyline) is
// extruded along a vector tilted by `helix_angle_deg` from the gear axis.
// This produces the correct projected tooth twist (top of tooth offset
// from bottom by Z * tan(helix)). For a true helical sweep with constant
// normal cross-section, callers should request a future helical_sweep
// variant — here we explicitly tag the synthesis method in tooling.extra.
//
// DFM gates per AGMA 2005-D03:
//   DFM-GEAR-MODULE    normal_module out of [0.5, 10] mm
//   DFM-GEAR-COUNT     N < 6 or N > 200
//   DFM-GEAR-HELIX     helix_angle not in [5, 45] deg
//   DFM-GEAR-OD        OD = (N+2)*m must fit blank
//   DFM-GEAR-FW        face_width >= 2 * normal_pitch (min 2 teeth engaged)
//
// Standard: AGMA 2005 / ISO 53

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace helical_gear_teeth {

constexpr const char* kSkillId = "helical_gear_teeth";

struct Input
{
    double  blank_dia_mm        = 0.0;
    double  blank_thick_mm      = 0.0;   // = face width
    double  normal_module_mm    = 0.0;
    double  pitch_dia_mm        = 0.0;
    double  helix_angle_deg     = 15.0;  // 5..45 typical
    double  pressure_angle_deg  = 20.0;
    double  bore_dia_mm         = 0.0;
    bool    right_hand          = true;  // helix direction
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace helical_gear_teeth
}  // namespace koocadcam::skill
