#pragma once
// @lat: [[engine/skills#sealed_bearing_seat_with_shield_relief]]
//
// sealed_bearing_seat_with_shield_relief — COMPOUND seat for a sealed
// (2RS / 2Z / 2RU) bearing whose shield protrudes axially beyond the
// nominal bearing width:
//
//   sub 1: outer bore at outer_dia (bearing OD seat)
//   sub 2: outer-race seat axial face (planar shoulder for race press-fit)
//   sub 3: front shield-relief groove (axial slot at entry side, dia =
//          shield_relief_dia)
//   sub 4: back  shield-relief groove (axial slot at far side)
//   sub 5: 30° lead-in chamfer
//
// Why two relief grooves: the contact-seal lip and the metal-shield ring
// on each side of a 2RS bearing extend ~0.5 mm beyond the bearing race —
// they would interfere with a tight seat surface, so a relief groove of
// shield_relief_dia (> outer_dia) is cut at each end of the bore so the
// shield rotates freely.  SKF nomenclature: "S" undercut on the seat.
//
// Standard relief dimensions (SKF guideline):
//   relief axial width  = 1.5 mm
//   relief radial depth = (shield_relief_dia - outer_dia) / 2 ≥ 0.3 mm
//
// DFM checks:
//   DFM-INPUT             : non-positive dims
//   DFM-SHIELD-DIA        : shield_relief_dia ≤ outer_dia (relief invalid)
//   DFM-SHIELD-RELIEF-MAX : shield_relief_dia > outer_dia + 2.0 (excessive
//                           relief; bearing won't seat)
//   DFM-002               : (shield_relief_dia − outer_dia)/2 < 0.3 mm
//                           (relief depth below useful minimum)
//
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sealed_bearing_seat_with_shield_relief {

constexpr const char* kSkillId = "sealed_bearing_seat_with_shield_relief";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm        = 0.0;
    double      position_y_mm        = 0.0;
    gp_Dir      axis_dir             { 0.0, 0.0, -1.0 };
    double      outer_dia_mm         = 0.0;
    double      shield_relief_dia_mm = 0.0;
    double      seat_depth_mm        = 0.0;    // axial extent of the bore
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sealed_bearing_seat_with_shield_relief
}  // namespace koocadcam::skill
