#pragma once
// @lat: [[engine/skills#gusset_plate_compound]]
//
// gusset_plate_compound — COMPOUND structural skill that builds a right-
// triangle gusset plate with bevelled fit edges, 4 weld-prep corner notches,
// and 1 lifting hole.
//
//   sub-feature 1: triangular plate body  (right-triangle, legs leg_a × leg_b)
//   sub-feature 2: chamfer cut #1 (bevel along the hypotenuse for fit)
//   sub-feature 3..6: 4 weld-prep corner notches (small radius rings at corners)
//   sub-feature 7: 1 lifting hole (drilled cylinder near plate centroid)
//
// subfeature_count = 1 + 1 + 4 + 1 = 7.
//
// Plate lies in the XY plane (z = 0 → z = plate_t).
// Right angle at the origin; legs run along +X (leg_a) and +Y (leg_b).
//
// DFM (AWS D1.1 + AISC J6 gusset connection geometry):
//   DFM-GUSSET-INPUT  : any non-positive input
//   DFM-GUSSET-THK    : plate_t < 6 mm (sub-structural plate)
//   DFM-GUSSET-LIFT-D : lift_hole_dia < 16 mm (shackle min size)
//   DFM-GUSSET-LIFT-E : lift hole edge distance < 2.5 × lift_hole_dia (AISC J3.5)

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gusset_plate_compound {

constexpr const char* kSkillId = "gusset_plate_compound";

struct Input
{
    double leg_a_mm        = 300.0;
    double leg_b_mm        = 300.0;
    double plate_t_mm      = 12.0;
    double lift_hole_dia_mm = 25.0;
    double bevel_mm        = 5.0;     // bevel size along the hypotenuse
    double notch_r_mm      = 8.0;     // weld-prep corner notch radius
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gusset_plate_compound
}  // namespace koocadcam::skill
