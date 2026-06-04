#pragma once
// @lat: [[engine/skills#beam_dump_aperture_blackbody]]
//
// beam_dump_aperture_blackbody — OPTICAL compound: beam dump with internal
// black-body cavity (multi-step bore + angled cone for absorption).
//
// Sub-features (3 ops):
//   1. cut aperture entry (small dia cylinder)            (CUT)
//   2. cut internal cylindrical chamber (light absorption)(CUT)
//   3. cut conical end (5–30° angle, increases absorption)(CUT)
//
// DFM:
//   DFM-INPUT       : face_id valid, all dims > 0
//   DFM-APERTURE    : aperture_dia_mm < internal_chamber_dia_mm
//   DFM-CONE-ANGLE  : cone_angle_deg ∈ [5, 30]
//   DFM-FACE        : face must be planar

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace beam_dump_aperture_blackbody {

constexpr const char* kSkillId = "beam_dump_aperture_blackbody";

struct Input
{
    int    face_id                   = -1;
    double center_x_mm               = 0.0;
    double center_y_mm               = 0.0;
    double aperture_dia_mm           = 5.0;
    double aperture_depth_mm         = 3.0;
    double internal_chamber_dia_mm   = 15.0;
    double internal_chamber_depth_mm = 18.0;
    double cone_angle_deg            = 12.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace beam_dump_aperture_blackbody
}  // namespace koocadcam::skill
