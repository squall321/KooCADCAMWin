#pragma once
// @lat: [[engine/skills#gmt_bezel_24hr]]
//
// gmt_bezel_24hr — COMPOUND FEATURE.
//
// Dual-time GMT 24-hour bezel: an annular bezel ring + N evenly-spaced hour
// markers cut on the outer rim every 15° (24 markers × 15° = 360°).  Used
// on Rolex GMT-Master and similar pilot/world-time watches.
//
// Sub-feature chain:
//   1. annular bezel ring (outer cyl − inner cyl)
//   2..N+1. hour marker cuts at i × (360°/N) around outer rim.
//
// DFM: outer > inner, hour_marker_count ∈ [12, 24], height > 0.3 mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gmt_bezel_24hr {

constexpr const char* kSkillId = "gmt_bezel_24hr";

struct Input
{
    FaceDatum   case_top_face;
    gp_Ax1      case_axis            { gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1) };
    double      outer_dia_mm  = 0.0;
    double      inner_dia_mm  = 0.0;
    double      height_mm     = 0.0;
    int         hour_marker_count = 24;
    double      marker_dia_mm  = 1.2;
    double      marker_depth_mm = 0.5;
    double      top_z_mm       = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gmt_bezel_24hr
}  // namespace koocadcam::skill
