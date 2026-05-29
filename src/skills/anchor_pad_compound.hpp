#pragma once
// @lat: [[engine/skills#anchor_pad_compound]]
//
// anchor_pad_compound — REAL multi-cut compound feature for a cast-in
// anchor pad (used at the base of equipment mounted to a concrete slab).
//
// Sub-features:
//   1. base plate (added cuboid pad)                    [fuse]
//   2. 4 stud projections (cylinders along +Z)          [fuse × 4]
//   3. central conduit hole (through-plate cylinder cut)[cut]
//   4. 4 leveling-nut counterbores (cuts on top face)   [cut × 4]
//
// subfeature_count = 1 + 4 + 1 + 4 = 10.
//
// DFM standard cited:
//   • ACI 318 / AISC 360 anchor design — min edge distance ≥ 1.5 × stud_dia,
//     stud spacing ≥ 3 × stud_dia, plate thickness ≥ 0.5 × stud_dia,
//     central conduit ≤ min(plate_dim)/3.
//
// Recognize:
//   1. Find ≥ 4 axis-parallel small cylindrical faces above a planar pad.
//   2. Confirm one larger central cylindrical face (the conduit).
//   3. Confirm counterbore step circles on top face.
//   Confidence 0.8 (geometric) / 1.0 (replay).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace anchor_pad_compound {

constexpr const char* kSkillId = "anchor_pad_compound";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm        = 0.0;
    double      center_y_mm        = 0.0;
    gp_Dir      axis_dir           { 0.0, 0.0, 1.0 };  // studs grow up
    double      plate_length_mm    = 240.0;
    double      plate_width_mm     = 240.0;
    double      plate_thickness_mm = 24.0;
    double      stud_dia_mm        = 20.0;
    double      stud_height_mm     = 60.0;
    double      stud_pitch_x_mm    = 160.0;            // centre-to-centre X
    double      stud_pitch_y_mm    = 160.0;            // centre-to-centre Y
    double      conduit_dia_mm     = 40.0;             // central through-hole
    double      leveling_nut_dia_mm = 32.0;            // counterbore dia (around each stud)
    double      leveling_nut_depth_mm = 6.0;
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace anchor_pad_compound
}  // namespace koocadcam::skill
