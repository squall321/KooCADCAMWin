#pragma once
// @lat: [[engine/skills#generator_stator_slot_pattern]]
//
// generator_stator_slot_pattern — Stator slot pattern for generator/motor
// lamination.  Wind turbine generators (PM synchronous or DFIG induction)
// have N radial slots cut around the stator inner bore to hold copper
// windings.  Utility-scale slot count: 36/48/72 (3-phase × poles/3).
//
// Slot profile: rectangular cross-section, open to the inner bore via a
// narrower "slot opening" (slot_opening_width < slot_width — keeps wire
// from migrating into the airgap).
//
// Sub-feature count = slot_count.  Each slot is a box cutter placed by
// gp_Trsf::SetRotation around the stator centerline and cut SEQUENTIALLY
// against the running workpiece.
//
// DFM standard cited:
//   IEC 60034 — rotating electrical machines; standard utility-scale
//   slot counts {36, 48, 72} for 3-phase 4/6/8-pole generators.  Slot
//   width × slot_count must be < π × stator_inner_dia (otherwise teeth
//   are physically impossible).
//
// Signature.pattern:
//   { kind = "generator_stator_slot_pattern",
//     is_compound = true,
//     wind_feature_type = "generator_stator_slot_pattern",
//     slot_count, stator_inner_dia_mm, slot_width_mm, slot_depth_mm,
//     slot_opening_width_mm }

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace generator_stator_slot_pattern {

constexpr const char* kSkillId = "generator_stator_slot_pattern";

struct Input
{
    int    face_id                  = -1;
    double center_x_mm              = 0.0;
    double center_y_mm              = 0.0;
    double stator_inner_dia_mm      = 600.0;
    double stator_outer_dia_mm      = 900.0;
    int    slot_count               = 48;
    double slot_width_mm            = 12.0;
    double slot_depth_mm            = 50.0;
    double slot_opening_width_mm    = 4.0;
    double stator_thickness_mm      = 100.0;  // axial stack length
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace generator_stator_slot_pattern
}  // namespace koocadcam::skill
