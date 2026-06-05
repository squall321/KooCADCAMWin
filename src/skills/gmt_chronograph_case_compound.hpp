#pragma once
// @lat: [[engine/skills#gmt_chronograph_case_compound]]
//
// gmt_chronograph_case_compound — VERY HIGH-ORDER COMPOUND ASSEMBLY.
//
// GMT chronograph wristwatch case with 3 chronograph sub-dial pockets +
// GMT 24-hour bezel + 2 chronograph pushers + crown.  Composes 5+ slice
// 12/13 watch skills sequentially.
//
// Sub-skill composition chain:
//   1. stock cylinder
//   2. 3 sub-dial pockets at fixed angular positions (3/6/9 o'clock dial face)
//      → uses pr::cylinder + pr::cutMany (annular sub-dial cavities)
//   3. crown_stem_cavity_compound at 3 o'clock
//   4. 2 chronograph pusher holes at 2 and 4 o'clock
//   5. diving_bezel_unidirectional configured as GMT 60-click (24h scale)
//   6. case_back_screw_down (AS568 -116 O-ring)
//
// Output: GMT chrono case body.  pattern.is_compound = true,
// pattern.is_watch_assembly = true, pattern.watch_style = "gmt_chronograph",
// subfeature_count = 7.
//
// DFM gates:
//   - case_dia_mm ∈ [38, 48] mm (GMT chrono typical range)
//   - case_height_mm ∈ [10, 17] mm
//   - subdial_count = 3 (chronograph hours/minutes/seconds)
//   - pusher_count = 2

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gmt_chronograph_case_compound {

constexpr const char* kSkillId = "gmt_chronograph_case_compound";

struct Input
{
    double      case_dia_mm     = 42.0;
    double      case_height_mm  = 14.0;
    int         subdial_count   = 3;   // 3 chrono subdials
    int         pusher_count    = 2;   // 2 chronograph pushers
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gmt_chronograph_case_compound
}  // namespace koocadcam::skill
