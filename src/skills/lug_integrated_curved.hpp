#pragma once
// @lat: [[engine/skills#lug_integrated_curved]]
//
// lug_integrated_curved — COMPOUND FEATURE (2 × lug_count sub-features).
//
// Watch-case integrated lugs welded directly to the case body (vs.
// classical detachable lugs).  N lug volumes placed in pairs around the
// case at +/- (lug_angle_deg/2) offsets from the 12-o'clock and 6-o'clock
// positions, each with a spring-bar through-hole.
//
// Sub-feature chain (lug_count iterations of fuse + cut):
//   1..lug_count: FUSE each curved lug box onto the case OD
//   lug_count+1..2*lug_count: CUT spring-bar through-hole through each lug
//
// Output: case body with `lug_count` lugs (default 4 = 2 pair), each with a
// transverse spring-bar hole.  Single FeatureSignature, pattern.is_compound =
// true, pattern.watch_feature_type = "integrated_curved_lugs".
//
// DFM gates (Bergeon catalog + service spec):
//   - lug_count = 4 (2 pair) OR 6 (3 pair) ONLY
//   - lug_angle_deg ∈ [60, 100] (typical strap width subtension)
//   - lug_thickness > 1.0 mm (mechanical)
//   - lug_length ≥ 4 × spring_bar_dia (bar end material)
//   - spring_bar_dia ∈ {1.2, 1.5, 1.6, 1.8, 2.0} mm
//   - spring_bar_z_offset places hole inside the lug volume with > 0.3 mm wall
//   - lug_width  ≥ 2 × spring_bar_dia
//
// Recognize: metadata replay (curved lug + transverse hole pattern).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lug_integrated_curved {

constexpr const char* kSkillId = "lug_integrated_curved";

struct Input
{
    FaceDatum   case_side_face;
    gp_Ax1      case_axis             { gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1) };
    double      case_outer_radius_mm = 0.0;
    int         lug_count            = 4;     // 4 (2 pair) or 6 (3 pair)
    double      lug_angle_deg        = 80.0;  // subtension between pair
    double      lug_width_mm         = 4.0;   // axial extent
    double      lug_thickness_mm     = 2.5;   // tangential
    double      lug_length_mm        = 7.0;   // radial protrusion
    double      spring_bar_dia_mm    = 1.5;
    double      spring_bar_z_offset_mm = 0.0; // 0 → at lug Z center
    double      lug_attach_top_z_mm  = 0.0;   // 0 → use case top
    double      lug_attach_bot_z_mm  = 0.0;   // 0 → use case bottom
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lug_integrated_curved
}  // namespace koocadcam::skill
