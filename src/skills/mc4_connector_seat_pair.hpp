#pragma once
// @lat: [[engine/skills#mc4_connector_seat_pair]]
//
// mc4_connector_seat_pair — SOLAR/PV compound: MC4 photovoltaic connector
// pair seat (positive + negative) on a junction-box / inverter housing
// face, with strain-relief slots at each connector for cable retention.
//
// Sub-feature chain (SEQUENTIAL pr::cut, 4 ops):
//   1. CUT connector bore #1 (positive lead) at pair_origin
//   2. CUT strain-relief slot #1 at connector #1
//   3. CUT connector bore #2 (negative lead) at pair_origin + (pair_pitch, 0)
//   4. CUT strain-relief slot #2 at connector #2
//
// Signature:
//   pattern.kind                = "mc4_connector_seat_pair"
//   pattern.is_compound         = true
//   pattern.solar_feature_type  = "mc4_connector_pair"
//   pattern.subfeature_count    = 4
//   pattern.connector_dia_mm
//   pattern.pair_pitch_mm
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   - all positive
//   - connector_dia in [5, 12] mm (MC4 family ~7 mm pin)
//   - pair_pitch >= connector_dia + 2 mm (bores cannot overlap)
//   - strain_relief slot fits between connector bores (no overlap)

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mc4_connector_seat_pair {

constexpr const char* kSkillId = "mc4_connector_seat_pair";

struct Input
{
    int    face_id                   = -1;
    double pair_origin_x_mm          = 0.0;
    double pair_origin_y_mm          = 0.0;
    double pair_pitch_mm             = 30.0;  // typical 25–35 mm
    double connector_dia_mm          = 7.0;   // MC4 ~7 mm pin housing
    double seat_depth_mm             = 6.0;
    double strain_relief_slot_w_mm   = 3.0;
    double strain_relief_slot_l_mm   = 8.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mc4_connector_seat_pair
}  // namespace koocadcam::skill
