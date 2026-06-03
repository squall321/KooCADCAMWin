#pragma once
// @lat: [[engine/skills#board_to_board_connector_seat]]
//
// board_to_board_connector_seat — PHONE compound: recess pattern for a
// board-to-board (BTB) connector on a phone middle frame.
//
// Sub-features (3 ops):
//   1. central rectangular pocket  (cut)   length × width × height
//   2. keep-out chamfer ring        (cut)   wider/shallow pocket around #1
//   3. chamfered top edges          (cut)   thin slope along the pocket rim
//
// DFM:
//   DFM-INPUT             : all dims > 0; keep_out_margin_mm ≥ 0
//   DFM-BTB-RANGE         : connector_length_mm ∈ [8, 25]; width ∈ [1.5, 2.5];
//                           height ∈ [0.6, 1.2] (typical phone BTB envelope)
//
// Signature pattern:
//   pattern.kind                       = "board_to_board_connector_seat"
//   pattern.is_compound                = true
//   pattern.is_phone_feature           = true
//   pattern.derived_volume_removed_mm3 = sum of cut volumes
//   pattern.connector_length_mm / connector_width_mm / connector_height_mm

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace board_to_board_connector_seat {

constexpr const char* kSkillId = "board_to_board_connector_seat";

struct Input
{
    int    face_id              = -1;
    double connector_position_x_mm = 0.0;
    double connector_position_y_mm = 0.0;
    double connector_length_mm   = 12.0;
    double connector_width_mm    = 2.0;
    double connector_height_mm   = 0.8;
    double keep_out_margin_mm    = 0.3;
    // ── Slice-13 phone PCB-mount extension (optional, backward-compatible) ──
    // If mounting_post_count > 0, N small cylindrical bosses are FUSED beside
    // the pocket (2-post → ends along length axis; 4-post → ends along both
    // length and width axes).  Posts have post_dia_mm OD and height equal to
    // the pocket depth (connector_height_mm) so they sit at the pocket floor.
    int    mounting_post_count  = 0;     // 0 | 2 | 4
    double post_dia_mm          = 0.0;   // > 0 required if mounting_post_count > 0
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace board_to_board_connector_seat
}  // namespace koocadcam::skill
