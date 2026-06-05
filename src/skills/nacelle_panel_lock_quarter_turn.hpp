#pragma once
// @lat: [[engine/skills#nacelle_panel_lock_quarter_turn]]
//
// nacelle_panel_lock_quarter_turn — Heavy-duty quarter-turn cam latch for
// wind turbine nacelle access panels (similar topology to Dzus but heavier
// duty for marine/onshore exposure with O-ring sealing).
//
// 4 sub-features cut SEQUENTIALLY against the running workpiece:
//   1. latch through-bore (cylindrical, latch_body_dia × through full wall)
//   2. wider receptacle pocket (concentric counterbore, latch_depth)
//   3. cam engagement undercut (annular ring beyond pocket — receives the
//      quarter-turn cam wings)
//   4. O-ring seal groove (AS568 spec — annular ring at entry face)
//
// Concentric overlapping cutters → SEQUENTIAL pr::cut required.
//
// DFM standard cited:
//   AS568 O-ring sizes for the seal groove; latch geometry per heavy-duty
//   panel-lock conventions (Southco / Camloc product family).
//
// Signature.pattern:
//   { kind = "nacelle_panel_lock_quarter_turn",
//     is_compound = true,
//     wind_feature_type = "nacelle_panel_lock_quarter_turn",
//     latch_body_dia_mm, latch_depth_mm, cam_engagement_depth_mm,
//     sealing_o_ring_size_key }

#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace nacelle_panel_lock_quarter_turn {

constexpr const char* kSkillId = "nacelle_panel_lock_quarter_turn";

struct Input
{
    int    face_id                  = -1;
    double center_x_mm              = 0.0;
    double center_y_mm              = 0.0;
    double latch_body_dia_mm        = 25.0;
    double latch_depth_mm           = 12.0;
    double cam_engagement_depth_mm  = 4.0;
    std::string sealing_o_ring_size_key = "-116";  // AS568 key
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace nacelle_panel_lock_quarter_turn
}  // namespace koocadcam::skill
