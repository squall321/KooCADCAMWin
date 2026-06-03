#pragma once
// @lat: [[engine/skills#tachymeter_bezel_engraved]]
//
// tachymeter_bezel_engraved — COMPOUND FEATURE (2 logical sub-features).
//
// Fixed (non-rotating) tachymeter bezel ring with engraved scale markings —
// thin radial slits forming major and minor index ticks around the bezel ring
// at the top of the case (chronograph-style: Daytona / Speedmaster).
//
// Sub-feature chain (FUSED then single cut):
//   1. major radial markings  — `major_count` thin radial rectangles cut into
//                               the bezel ring at evenly-spaced angles
//   2. minor radial markings  — `minor_count` shorter & thinner ticks between
//                               the majors (typically 4× minor per 1× major)
//
// Output: case top with engraved radial tick pattern.  Single FeatureSignature,
// pattern.is_compound = true, pattern.watch_feature_type = "tachymeter_scale".
//
// DFM gates:
//   - bezel_outer_dia > bezel_inner_dia (positive ring)
//   - engrave_depth ∈ [0.05, 0.8] mm (legible without weakening ring)
//   - major_count ∈ {12, 24, 60} (common tachymeter step counts)
//   - minor_count divisible by major_count and ≤ 240
//   - clearance between adjacent ticks (at inner radius) ≥ 0.3 mm
//
// Recognize: metadata replay (radial line pattern detectable but not in slice 1).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tachymeter_bezel_engraved {

constexpr const char* kSkillId = "tachymeter_bezel_engraved";

struct Input
{
    FaceDatum   case_top_face;
    gp_Ax1      case_axis             { gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1) };
    double      bezel_outer_dia_mm  = 0.0;
    double      bezel_inner_dia_mm  = 0.0;
    double      engrave_depth_mm    = 0.15;
    int         major_count         = 12;
    int         minor_count         = 48;
    double      major_tick_width_mm = 0.4;
    double      minor_tick_width_mm = 0.2;
    double      major_tick_inset_mm = 0.0;   // inward extension from outer Ø
    double      top_z_mm            = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tachymeter_bezel_engraved
}  // namespace koocadcam::skill
