#pragma once
// @lat: [[engine/skills#helical_gear]]
//
// helical_gear — REAL helical involute gear (slice 13).
//
// Same as spur_gear_involute but each tooth gap is rotated incrementally
// along the face_width by helix_angle_deg / face_width.  Implemented as
// N axial slices, each slice rotated by Δθ per slice — the "stacked slabs"
// approximation of a true helical sweep.
//
// Helix rotation across face:
//   Δθ_total = (face_width / r_pitch) · tan(β),  β = helix_angle
//
// DFM checks:
//   DFM-GEAR-MOD     module_mm > 0.3
//   DFM-GEAR-CT      teeth_count in [8, 200]
//   DFM-GEAR-FW      face_width > 0
//   DFM-GEAR-PA      pressure_angle in [14.5, 30]
//   DFM-GEAR-HEL     helix_angle in [0, 35] deg

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace helical_gear {

constexpr const char* kSkillId = "helical_gear";

struct Input
{
    double  module_mm           = 2.0;
    int     teeth_count         = 20;
    double  face_width_mm       = 10.0;
    double  pressure_angle_deg  = 20.0;
    double  helix_angle_deg     = 15.0;
    double  blank_outer_dia_mm  = 0.0;   // 0 → auto = 2·r_tip
    int     axial_slice_count   = 2;     // # of stacked slabs for helix approx;
                                         // default 2 = "fast preview" mode
                                         // (z×slabs cutter Booleans dominate cost;
                                         // raise to 4–6 for production-grade twist).
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace helical_gear
}  // namespace koocadcam::skill
