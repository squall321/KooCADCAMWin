#pragma once
// @lat: [[engine/skills#diving_bezel_120clicks]]
//
// diving_bezel_120clicks — COMPOUND FEATURE (3+ primitive ops, watch-side).
//
// Unidirectional diving bezel: an annular ring CUT from the case top to
// form the bezel seat, plus N small triangular click notches (approximated
// as small cylinder cuts) replicated around the outer rim at 360°/N spacing.
//
// Standard ISO 6425 dive watches use 120 clicks (3° per click).  Range is
// 60-240 (every 6° down to every 1.5°).
//
// Sub-feature chain:
//   1. annular bezel ring   — outer cylinder cut + inner cylinder return
//                             (annularRing) at top of case
//   2..N+1. click notches   — N small cylinder cuts replicated around the
//                             outer rim (rotation about case axis)
//
// Output: case with bezel seat and N notches around the rim.  Single
// FeatureSignature, pattern.is_compound = true, subfeature_count = 2 (logical
// groups: bezel + clicks), pattern.click_count = N, pattern.is_watch_feature.
//
// DFM gates:
//   - outer_dia > inner_dia (bezel has non-zero radial extent)
//   - click_count ∈ [60, 240]
//   - all dims > 0
//   - height > 0.3 mm (machinable bezel seat)
//
// Recognize: metadata replay (1.0 confidence) — annular face + circular click
// pattern can be detected geometrically in future slices.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace diving_bezel_120clicks {

constexpr const char* kSkillId = "diving_bezel_120clicks";

struct Input
{
    FaceDatum   case_top_face;       // top of case (where bezel sits)
    gp_Ax1      case_axis            { gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1) };
    double      outer_dia_mm  = 0.0;
    double      inner_dia_mm  = 0.0;
    double      height_mm     = 0.0;  // depth of bezel seat
    int         click_count   = 120;
    double      notch_dia_mm  = 0.6;  // small notch cylinder Ø
    double      notch_depth_mm = 0.5;
    double      top_z_mm      = 0.0;  // top face Z; 0 -> infer from bbox
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace diving_bezel_120clicks
}  // namespace koocadcam::skill
