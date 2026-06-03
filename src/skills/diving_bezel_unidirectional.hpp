#pragma once
// @lat: [[engine/skills#diving_bezel_unidirectional]]
//
// diving_bezel_unidirectional — COMPOUND FEATURE (2 logical sub-features).
//
// Watch-side ISO 6425 unidirectional diving bezel SEAT cut into the case top.
// Differs from the existing diving_bezel_120clicks skill in that the click
// notches sit on the seat OD ring (the ratchet teeth that engage the spring-
// loaded pawl underneath the bezel ring) rather than around the outer rim.
// Click count is restricted to either 60 (one per minute, GMT-style) or 120
// (every 3°, ISO 6425 dive-spec).
//
// Sub-feature chain (FUSED then single cut):
//   1. annular bezel seat ring  — outer cylinder minus inner cylinder cut
//                                 on the case top face, depth = bezel_height
//   2..N+1. ratchet click notches replicated around the seat OD at radius
//          (bezel_outer_dia + bezel_inner_dia) / 4 + bezel_outer_dia / 4 ≈
//          seat OD edge.  Each notch is a small cylinder cut to click_depth.
//
// Output: case top with annular seat + N notches around the seat OD.  Single
// FeatureSignature, pattern.is_compound = true, pattern.is_watch_feature = true,
// pattern.watch_feature_type = "diving_bezel_seat_with_ratchet".
//
// DFM gates:
//   - bezel_outer_dia > bezel_inner_dia (positive radial extent)
//   - radial extent ≥ 1.5 mm (machinable seat width)
//   - click_count ∈ {60, 120}  (ISO 6425 / GMT spec)
//   - click_depth < bezel_height (notch doesn't punch through seat floor)
//   - all dimensions > 0
//
// Recognize: metadata replay (geometric concentric ring + ratchet polar array
// detection deferred to future slices).  Confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace diving_bezel_unidirectional {

constexpr const char* kSkillId = "diving_bezel_unidirectional";

struct Input
{
    FaceDatum   case_top_face;
    gp_Ax1      case_axis            { gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1) };
    double      bezel_outer_dia_mm = 0.0;
    double      bezel_inner_dia_mm = 0.0;
    double      bezel_height_mm    = 0.0;
    int         click_count        = 120;
    double      click_depth_mm     = 0.4;
    double      click_notch_dia_mm = 0.5;
    double      top_z_mm           = 0.0;   // 0 → infer from bbox
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace diving_bezel_unidirectional
}  // namespace koocadcam::skill
