#pragma once
// @lat: [[engine/skills#zif_connector_window_compound]]
//
// zif_connector_window_compound — PHONE compound: ZIF (zero insertion force)
// flex cable connector window through a phone metal frame.
//
// Sub-features (compound, 3 cuts):
//   1. central through-window  — rectangular pocket cut completely through
//                                window_length × window_width × window_depth
//                                (depth_mm; if it exceeds frame thickness the
//                                cut is a through cut).
//   2. left  locking-tab recess — small side recess for the flex-actuator latch
//   3. right locking-tab recess — mirror of (2)
//
// DFM:
//   DFM-INPUT      : all dims > 0
//   DFM-MIN-WALL   : window_depth ≥ 0.4 mm (tab pocket wall thickness)
//   DFM-MIN-HOLE   : window_width_mm ≥ 0.3 mm (cuttable on a small endmill)
//   DFM-PHONE-RNG  : window_length ∈ [4, 25]; window_width ∈ [0.8, 2.5]
//
// Signature pattern:
//   pattern.kind                       = "zif_connector_window_compound"
//   pattern.is_compound                = true
//   pattern.is_phone_feature           = true
//   pattern.phone_feature_type         = "zif_window"
//   pattern.subfeature_count           = 3
//   pattern.window_length/width/depth_mm
//   pattern.derived_volume_removed_mm3

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace zif_connector_window_compound {

constexpr const char* kSkillId = "zif_connector_window_compound";

struct Input
{
    int    face_id              = -1;
    double center_x_mm          = 0.0;
    double center_y_mm          = 0.0;
    double window_length_mm     = 12.0;
    double window_width_mm      = 1.6;
    double window_depth_mm      = 1.0;
    double locking_tab_width_mm = 1.0;
    double locking_tab_depth_mm = 0.5;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace zif_connector_window_compound
}  // namespace koocadcam::skill
