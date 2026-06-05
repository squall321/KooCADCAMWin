#pragma once
// @lat: [[engine/skills#tractor_pto_spline_compound]]
//
// tractor_pto_spline_compound — Tractor Power Take-Off spline shaft per
// ASAE S203 (slice 16, agricultural/forestry).
//
// PTO spline standards (ASAE S203.14 / ISO 500):
//   - Type 1: 6-spline, 1-3/8" nominal dia (34.92 mm), 540 RPM
//   - Type 2: 21-spline, 1-3/8" nominal dia (34.92 mm), 1000 RPM
//   - Type 3: 20-spline, 1-3/4" nominal dia (44.45 mm), 1000 RPM
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   For each of N spline slots i in [0, N):
//     1. Compute angle theta_i = 2π·i / N
//     2. Build slot cutter (radial wedge) at that angle
//     3. pr::cut(current, slot_i)  — applied one at a time
//
// subfeature_count = N (slot count from pto_type).
//
// DFM:
//   DFM-INPUT    : positive dims required.
//   DFM-PTO-TYPE : pto_type must be "type1_6spline" | "type2_21spline" |
//                  "type3_20spline".
//   DFM-PTO-DIA  : shaft_dia must match the nominal for the chosen type
//                  within ±1 mm (caller may use rough stock + finish later).
//   DFM-PTO-LEN  : spline_length > 0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tractor_pto_spline_compound {

constexpr const char* kSkillId = "tractor_pto_spline_compound";

struct Input
{
    FaceDatum   face_id;
    std::string pto_type;                       // "type1_6spline" | "type2_21spline" | "type3_20spline"
    double      shaft_dia_mm    = 34.92;        // nominal per type
    double      spline_length_mm = 50.0;
    gp_Pnt      shaft_origin    { 0.0, 0.0, 0.0 };  // bottom-centre of shaft
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tractor_pto_spline_compound
}  // namespace koocadcam::skill
