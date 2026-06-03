#pragma once
// @lat: [[engine/skills#tachymeter_bezel_scale]]
//
// tachymeter_bezel_scale — COMPOUND FEATURE (annular bezel + engraved radial
// tachymeter markings).
//
// Fixed bezel with tachymeter scale.  A tachymeter measures speed by lap
// time (km/h): scale = 3600 / t_sec, where t_sec is elapsed time at the
// position around the bezel.  The angular position for a given speed value
// is theta_deg = 360 · (t_sec / 60_sec) = 360 · 3600 / (60 · speed) =
// 21600 / speed → tachymeter spacing is LOGARITHMIC in angle vs. speed.
//
// We approximate by spacing `marking_count` engraved radial lines between
// 60 km/h and 400 km/h, with angles computed from the tachymeter formula.
//
// Sub-feature chain:
//   1. annular bezel ring (outer cyl − inner cyl, depth = height_mm)
//   2..N+1. shallow radial line marks at tachymeter-calibrated angles
//
// DFM: outer > inner, marking_depth > 0, marking_count in [10, 240].

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tachymeter_bezel_scale {

constexpr const char* kSkillId = "tachymeter_bezel_scale";

struct Input
{
    FaceDatum   case_top_face;
    gp_Ax1      case_axis            { gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1) };
    double      outer_dia_mm  = 0.0;
    double      inner_dia_mm  = 0.0;
    double      height_mm     = 0.0;
    double      marking_depth_mm  = 0.2;
    int         marking_count = 60;
    double      marking_width_mm  = 0.3;  // thin line cylinder Ø
    double      top_z_mm      = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tachymeter_bezel_scale
}  // namespace koocadcam::skill
