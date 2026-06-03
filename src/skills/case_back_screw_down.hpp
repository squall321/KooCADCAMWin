#pragma once
// @lat: [[engine/skills#case_back_screw_down]]
//
// case_back_screw_down — COMPOUND FEATURE (3 sub-features).
//
// Screw-down case back with O-ring face/radial seal — the watch's primary
// water-resistance feature.  Combines:
//
//   1. internal cosmetic thread band  — a shallow annular ring cut on the
//      inner wall of the caseback at the case-body interface, dimensions
//      drawn from thread_pitch / thread_depth (the actual helix is metadata-
//      only and scheduled by thread_mill_internal downstream)
//   2. O-ring groove (flat OR external)
//        - "flat"     : annular face-seal groove on the caseback inner top
//        - "external" : radial groove on the cylindrical wall just below
//                       the thread band
//      Both groove geometries take width / depth from the AS568 central
//      table (_as568_table.hpp) by dash-number.
//   3. caseback OD seat shoulder (small annular relief at the top of the
//      thread band so the O-ring has a clean seat)
//
// Output: caseback shape with thread-band groove + O-ring groove + relief.
// Single FeatureSignature, pattern.is_compound = true, pattern.is_watch_feature
// = true, pattern.watch_feature_type = "screw_down_caseback".
//
// DFM gates (ISO 22810 + AS568):
//   - back_dia_mm > 12 mm (small wristwatch minimum)
//   - thread_pitch_mm ∈ [0.4, 1.5] mm (M-spec micro range)
//   - thread_depth_mm ∈ [0.3, 1.2] mm
//   - O-ring size MUST be in AS568 central table
//   - O-ring groove fits within caseback wall (radial) or back (axial)
//   - clearance > 0.3 mm between groove and case OD/ID
//
// Recognize: metadata replay (annular grooves + helix metadata).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace case_back_screw_down {

constexpr const char* kSkillId = "case_back_screw_down";

struct Input
{
    FaceDatum    case_bottom_face;
    gp_Ax1       case_axis              { gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1) };
    double       back_dia_mm        = 0.0;     // back OD (== caseback OD)
    double       thread_pitch_mm    = 0.6;
    double       thread_depth_mm    = 0.5;
    double       thread_band_axial_mm = 1.5;   // axial length of thread band
    std::string  o_ring_groove_position = "flat";  // "flat" | "external"
    std::string  o_ring_size_key    = "-016";  // AS568 dash from central table
    double       bottom_z_mm        = 0.0;     // 0 → infer from bbox
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace case_back_screw_down
}  // namespace koocadcam::skill
