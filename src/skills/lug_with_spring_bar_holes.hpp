#pragma once
// @lat: [[engine/skills#lug_with_spring_bar_holes]]
//
// lug_with_spring_bar_holes — COMPOUND FEATURE (5 primitive ops).
//
// Watch-case strap lug: an additive rectangular protrusion off the case wall
// at a specified angular position around the case, with chamfered corners and
// two coaxial cross-bores through the lug ends to receive a Ø 1.5 mm spring
// bar.  Chains:
//
//   1. fuse a rectangular lug box onto the case body (radially outward)
//   2. chamfer 2 outer top edges of the lug box (approximated as small cuts)
//   3. cut spring-bar through hole #1 (axis = horizontal tangential)
//   4. cut spring-bar through hole #2 (coaxial with #1, opposite lug end)
//
// Output: case body + lug protrusion with two through-holes.  Single
// FeatureSignature, pattern.is_compound = true, subfeature_count = 4.
//
// DFM gates (Bergeon / Seiko service-spec watch lug):
//   - spring_bar_dia ∈ {1.5, 1.6, 1.8} mm (standard catalog)
//   - lug_length ≥ 4·spring_bar_dia (enough material around bar)
//   - lug_width  ≥ 2·spring_bar_dia (enough wall thickness past hole)
//   - lug_length ≤ 16 mm (watch-case proportions)
//
// Recognize: metadata replay (compound additive+subtractive feature; not
// geometrically searchable in slice 1).  Confidence 1.0 when matched from
// wp.features() history.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lug_with_spring_bar_holes {

constexpr const char* kSkillId = "lug_with_spring_bar_holes";

struct Input
{
    gp_Ax1   case_axis;                  // case symmetry axis (e.g., +Z)
    double   case_outer_radius_mm = 0.0; // current case outer radius at lug height
    double   lug_angle_deg        = 0.0; // angular position around case (0 = +X)
    double   lug_attach_z_mm      = 0.0; // Z of lug centerline on the case
    double   lug_length_mm        = 8.0; // radial protrusion length
    double   lug_width_mm         = 4.0; // thickness across the lug (axial)
    double   lug_thickness_mm     = 2.5; // tangential (face) thickness
    double   corner_chamfer_mm    = 0.4; // edge chamfer on outer corners
    double   spring_bar_dia_mm    = 1.5; // through-hole Ø (Bergeon catalog)
    double   spring_bar_inset_mm  = 1.5; // axial distance from lug end to hole center
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lug_with_spring_bar_holes
}  // namespace koocadcam::skill
