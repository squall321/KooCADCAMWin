#pragma once
// @lat: [[engine/skills#valve_seat_insert_compound]]
//
// valve_seat_insert_compound — AUTOMOTIVE ICE engine compound: machined
// pocket in the cylinder head that receives a press-fit valve seat insert,
// finished with a 3-angle valve seat profile (top / seat / throat).
//
// Sub-feature chain (4 ops):
//   1. CUT cylindrical pocket for press-fit insert         (P7 hole-basis
//      interference fit from central _iso286_fits.hpp).
//   2. CUT top relief cone     — typical 30 deg (above seat).
//   3. CUT seat cone           — typical 45 deg (the actual sealing face).
//   4. CUT throat relief cone  — typical 60 deg (below seat, opens to port).
//
// Signature:
//   pattern.kind                  = "valve_seat_insert_compound"
//   pattern.is_compound           = true
//   pattern.engine_feature_type   = "valve_seat_insert"
//   pattern.subfeature_count      = 4
//   pattern.seat_press_fit_*      — actual P7 bore tolerance band
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   - face_id in range
//   - valve_head_dia_mm in [20, 60] mm (SAE J intake/exhaust valve range)
//   - seat_press_fit_dia_mm > valve_head_dia_mm (seat OD bigger than valve)
//   - all angles in (0, 90)
//   - seat_depth_mm positive and not larger than 0.6 * seat_press_fit_dia_mm

#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace valve_seat_insert_compound {

constexpr const char* kSkillId = "valve_seat_insert_compound";

struct Input
{
    int     face_id              = -1;
    double  center_x_mm          =  0.0;
    double  center_y_mm          =  0.0;
    double  valve_head_dia_mm    = 30.0;   // intake ~28-36, exhaust ~24-32
    double  seat_press_fit_dia_mm = 34.0;  // bore is P7 of this nominal
    double  seat_depth_mm        =  6.0;
    double  top_angle_deg        = 30.0;   // typ 30
    double  seat_angle_deg       = 45.0;   // typ 45 — main sealing angle
    double  throat_angle_deg     = 60.0;   // typ 60
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace valve_seat_insert_compound
}  // namespace koocadcam::skill
