#pragma once
// @lat: [[engine/skills#spur_gear_with_real_teeth]]
//
// spur_gear_with_real_teeth — REAL compound feature:
//
//   sub-cuts (N + 1):
//     1. central through bore (Ø bore_dia)
//     2..N+1. N involute tooth-flank cutouts removed radially from a
//             cylindrical blank, equispaced around the pitch circle.
//
// Tooth count N is derived from module + pitch_dia:
//     N = round(pitch_dia / module)             [ISO 53 standard]
//     pitch_dia = N * module
//
// Each tooth flank is approximated by a 5-point polyline (involute
// approximation) extruded full-thickness then cut. We approximate by
// removing wedge-shaped sectors between adjacent teeth — i.e. the
// *gullet* (space between teeth) is the cut, not the tooth.
//
// DFM gates per AGMA 2000 / ISO 1328:
//   DFM-GEAR-MODULE   module out of [0.5, 10] mm
//   DFM-GEAR-COUNT    N < 6 or N > 200
//   DFM-GEAR-PA       pressure_angle not in {14.5, 20, 25}
//   DFM-GEAR-OD       OD = (N + 2) * module must fit blank
//   DFM-GEAR-BORE     bore_dia too small / too big
//
// Standard: ISO 53 (cylindrical gears for general engineering)

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spur_gear_with_real_teeth {

constexpr const char* kSkillId = "spur_gear_with_real_teeth";

struct Input
{
    double  blank_dia_mm        = 0.0;   // raw stock OD (must >= OD = (N+2)*m)
    double  blank_thick_mm      = 0.0;   // face width
    double  module_mm           = 0.0;   // ISO module m
    double  pitch_dia_mm        = 0.0;   // d = N * m
    double  pressure_angle_deg  = 20.0;  // 14.5 / 20 / 25
    double  bore_dia_mm         = 0.0;   // central through bore
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace spur_gear_with_real_teeth
}  // namespace koocadcam::skill
