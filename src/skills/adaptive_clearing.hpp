#pragma once
// @lat: [[engine/skills#adaptive_clearing]]
//
// adaptive_clearing — modern CAM roughing strategy that keeps the cutter
// fully engaged in stock at a constant optimal load.  Stepover varies
// adaptively to maintain a target engagement percentage.  Like trochoidal,
// but works on arbitrary pocket footprints (not just trochoid-friendly
// shapes) and aggressively pushes feed/speed for productivity.
//
//                       entry_face (planar)
//                              │
//                  ────────────┼──────────────
//                              │
//                   ┌──────────┴──────────┐    ↑ depth_mm
//                   │ ▒ ▒ ▒ ▒ ▒ ▒ ▒ ▒     │    │  (adaptive engagement; the
//                   │ ▒ ▒ ▒ ▒ ▒ ▒ ▒ ▒     │    │   tool stays at the target
//                   │ ▒ ▒ ▒ ▒ ▒ ▒ ▒ ▒     │    │   engagement % of its dia)
//                   └─────────────────────┘    ↓
//                          length_mm
//
// Geometrically identical to `mill_rect_pocket`; the strategy difference
// lives in the signature.pattern.kind + tooling metadata:
//   - tool_type = "end_mill_adaptive"
//   - cutting_speed_sfm × 2 (most aggressive of the 5 strategies)
//   - feed_per_tooth_mm × 3 (very high chip load tolerated by constant
//     engagement model)
//   - optimal_engagement_pct recorded for CAM toolpath generation
//
// Signature pattern: rect pocket topology with
// `pattern.kind == "adaptive_clearing"`.
//
// DFM checks:
//   DFM-002       : tool_dia_mm ≥ 1.0 (adaptive needs cutter stiffness)
//   DFM-TROCH-SO  : stepover_mm ≤ tool_dia × 0.3 (the adaptive stepover
//                   peak — same upper bound as trochoidal)
//   DFM-ADAPT-ENG : optimal_engagement_pct ∈ [10, 60] (outside is unphysical
//                   for adaptive control: below 10 % is wasted toolpath,
//                   above 60 % the constant-load math breaks down)
//
// Recognize:
//   Topology overlaps with mill_rect_pocket / trochoidal_mill — geometric
//   differentiation is impossible.  Same scheme as trochoidal: prefer
//   feature-history (conf 0.90), fall back to a 0.40-confidence marker.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace adaptive_clearing {

constexpr const char* kSkillId = "adaptive_clearing";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm             = 0.0;
    double      center_y_mm             = 0.0;
    gp_Dir      axis_dir                { 0.0, 0.0, -1.0 };
    double      length_mm               = 0.0;
    double      width_mm                = 0.0;
    double      depth_mm                = 0.0;
    double      corner_r_mm             = 1.0;
    double      tool_dia_mm             = 6.0;
    // Trochoid-like stepover advance.  0 → tool_dia × 0.1 default.
    double      stepover_mm             = 0.0;
    // Target percent of the tool diameter that stays engaged in stock at
    // any instant.  Default 30 %.
    double      optimal_engagement_pct  = 30.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace adaptive_clearing
}  // namespace koocadcam::skill
