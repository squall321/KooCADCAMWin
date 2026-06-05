#pragma once
// @lat: [[engine/skills#ag_implement_3point_hitch_pin]]
//
// ag_implement_3point_hitch_pin — Cat I/Cat II/Cat III 3-point hitch pin
// mounting per ASAE S217 (slice 16, agricultural/forestry).
//
// The 3-point hitch is the universal tractor-to-implement interface.  Pin
// sizes are standardized by Category:
//
//   - Cat 1 (small tractors, ≤ 35 PTO HP): pin dia 19.0 mm (3/4")
//   - Cat 2 (medium, 30–95 PTO HP):        pin dia 25.4 mm (1")
//   - Cat 3 (large, 80+ PTO HP):           pin dia 31.8 mm (1-1/4")
//
// Each pin attachment has a cross-pin (linch pin) hole drilled perpendicular
// to the main pin axis for retention.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. primary hitch pin bore (Z-axis cylinder, full thickness)
//   2. cross linch-pin hole (Y-axis cylinder, perpendicular)
//
// subfeature_count = 2.
//
// DFM:
//   DFM-INPUT     : positive dims required.
//   DFM-CAT       : hitch_category in {1, 2, 3}.
//   DFM-POSITION  : pin_position in {"top_link", "lower_link"}.
//   DFM-LINCH     : linch_pin_hole_dia > 0 and < pin_dia / 2.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ag_implement_3point_hitch_pin {

constexpr const char* kSkillId = "ag_implement_3point_hitch_pin";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy            { 0.0, 0.0, 0.0 };
    int         hitch_category       = 1;                // 1 | 2 | 3
    std::string pin_position;                            // "top_link" | "lower_link"
    double      linch_pin_hole_dia_mm = 6.0;             // typ 5-8 mm linch pin
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ag_implement_3point_hitch_pin
}  // namespace koocadcam::skill
