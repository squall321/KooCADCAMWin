#pragma once
// @lat: [[engine/skills#iris_aperture_slot_array]]
//
// iris_aperture_slot_array — OPTICAL compound: iris/aperture leaf slot ring.
//
// Sub-features (1 + N ops):
//   1. cut central aperture bore                 (CUT)
//   2..(1+N). cut N angular slots evenly around  (CUT)
//
// DFM:
//   DFM-INPUT       : face_id valid, all dims > 0
//   DFM-SLOT-COUNT  : slot_count ∈ [4, 36]
//   DFM-SLOT-RADIUS : slot_outer_radius > slot_inner_radius > bore_dia/2
//   DFM-FACE        : face must be planar

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace iris_aperture_slot_array {

constexpr const char* kSkillId = "iris_aperture_slot_array";

struct Input
{
    int    face_id              = -1;
    double center_x_mm          = 0.0;
    double center_y_mm          = 0.0;
    double bore_dia_mm          = 6.0;
    int    slot_count           = 12;
    double slot_inner_radius_mm = 5.0;
    double slot_outer_radius_mm = 9.0;
    double slot_width_mm        = 0.6;
    double slot_depth_mm        = 1.5;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace iris_aperture_slot_array
}  // namespace koocadcam::skill
