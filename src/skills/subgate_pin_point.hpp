#pragma once
// @lat: [[engine/skills#subgate_pin_point]]
//
// subgate_pin_point — Sub-gate (pin-point) gate at the cavity entry.  This
// is a small conical channel that tapers down from the runner to the cavity
// and snaps off cleanly, followed by a tiny cylindrical "land" right at the
// parting line that becomes the witness mark on the part.
//
//      runner side
//          │   ◄── runner_dia  (taper top)
//          │
//          ▼ taper_angle_deg
//        ╱   ╲
//       ╱     ╲     conical sub-gate
//      ╱       ╲    length determined by taper_angle + gate_dia
//      └───────┘    ◄── gate_dia (taper bottom)
//      │       │
//      │  land │    short cylindrical land
//      │       │    (cavity_entry; land_length_mm)
//      └───────┘
//          │
//      cavity entry
//
// Implementation:
//   1. Compute taper height H = (runner_dia/2 - gate_dia/2) / tan(taper_angle).
//   2. Build conical cutter (BRepPrimAPI_MakeCone) from runner_dia at top
//      to gate_dia at bottom over height H.
//   3. Build cylindrical "land" of diameter gate_dia × land_length_mm.
//   4. Fuse cone+cylinder into a compound and cut from workpiece.
//
// Signature pattern:
//   pattern.kind                    = "subgate_pin_point"
//   pattern.is_compound             = true
//   pattern.mold_feature_type       = "subgate_pin_point"
//   pattern.derived_volume_removed  = cone+land volume
//
// DFM:
//   DFM-INPUT          : gate_dia / land_length > 0
//   DFM-SUBGATE-DIA    : gate_dia ∈ [0.3, 2.0] mm
//   DFM-SUBGATE-TAPER  : taper_angle ∈ [3, 15] deg
//   DFM-SUBGATE-LAND   : land_length ≥ 0.5 mm

#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace subgate_pin_point {

constexpr const char* kSkillId = "subgate_pin_point";

struct Input
{
    double  cavity_entry_x_mm = 0.0;
    double  cavity_entry_y_mm = 0.0;
    double  gate_dia_mm       = 0.0;
    double  land_length_mm    = 0.0;
    double  taper_angle_deg   = 5.0;     // 3-15
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace subgate_pin_point
}  // namespace koocadcam::skill
