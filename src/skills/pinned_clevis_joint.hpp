#pragma once
// @lat: [[engine/skills#pinned_clevis_joint]]
//
// pinned_clevis_joint — REAL multi-cut compound feature for a yoke-style
// clevis (the "U" half of a clevis–pin pair).
//
// Sub-features (REAL geometric cuts on a base block):
//   1. central tongue pocket  (rect slot down the centre of the U)
//   2. cross-pin hole         (transverse through-hole on both ears)
//   3. cotter-pin slot left   (radial small through-hole — left ear)
//   4. cotter-pin slot right  (radial small through-hole — right ear)
//
// subfeature_count = 4 (≥ 2 enforced).
//
// DFM standard cited:
//   • MIL-DTL-23949 / DIN 71752 — clevis bore-to-pin clearance 0.05–0.10 mm,
//     ear-thickness ≥ 0.6 × pin dia, cotter hole ≥ 0.8 mm and ≥ pin-dia/4.
//
// Recognize:
//   1. Find a transverse cylindrical face whose axis is perpendicular to
//      the dominant slot direction — that's the pin hole.
//   2. Confirm a rect pocket (planar bottom + 4 walls) whose long axis is
//      perpendicular to the pin hole axis.
//   3. Two small cylindrical faces parallel to the pin hole on each ear.
//   Confidence 0.8 (geometric) / 1.0 (replay).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pinned_clevis_joint {

constexpr const char* kSkillId = "pinned_clevis_joint";

struct Input
{
    FaceDatum   entry_face;
    // Center of the clevis "U" on the top face (axis_dir = -Z slot down).
    double      center_x_mm     = 0.0;
    double      center_y_mm     = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };  // slot direction
    double      block_length_mm = 60.0;              // dimension along pin axis
    double      block_width_mm  = 40.0;              // dimension transverse
    double      block_height_mm = 40.0;              // overall U-block depth
    double      tongue_slot_width_mm  = 14.0;        // slot width (allows tongue thickness)
    double      tongue_slot_depth_mm  = 26.0;        // slot depth (≤ block_height)
    double      pin_hole_dia_mm  = 12.0;             // cross-pin diameter
    double      pin_hole_z_offset_mm = 14.0;         // pin axis below top face
    double      cotter_pin_dia_mm = 3.0;             // cotter pin small hole
    double      cotter_pin_offset_mm = 10.0;         // cotter offset from pin axis along block_length
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pinned_clevis_joint
}  // namespace koocadcam::skill
