#pragma once
// @lat: [[engine/skills#eccentric_bushing_seat]]
//
// eccentric_bushing_seat — COMPOUND MACRO for an eccentric-bushing receiver.
//
// Sub-feature chain (4 sub-cuts):
//   1. Bushing bore                     — cylinder of `bore_od` for the
//                                         bushing OD seat (deep through the
//                                         face by bore_depth_mm).
//   2. Adjustment arc slot, CCW side    — narrow arc slot along the bore
//                                         rim of total arc `slot_arc_deg`
//                                         (typically 30°–90°), located
//                                         centred at angle 0°.
//   3. Adjustment arc slot, CW side     — same dimensions, located at
//                                         180° opposed (allows cam to
//                                         pivot ±slot_arc_deg/2).
//   4. Retention chamfer                — 0.5 mm × 45° cone frustum at the
//                                         entry rim, biases bushing nose
//                                         into the bore and prevents
//                                         knife-edge corrosion.
//
// (slice-1 implementation note: each "arc slot" is approximated as a small
// rectangular pocket tangent to the bore circle at the slot mid-angle; the
// real arc footprint is the swept area of a small end-mill.  The recorded
// `slot_arc_deg` lets CAM regenerate the proper toolpath.)
//
//                       ┌──── entry_face ────┐
//                       │                    │
//                       │   ┌────────────┐   │
//                       │   │   BORE     │   │
//                       │   │   (OD)     │   │
//                       │   └────────────┘   │
//                       │  ▲                ▲│
//                       │  │ slot A (CCW)   ││ slot B (CW)
//                       │                    │
//                       └────────────────────┘
//
// DFM:
//   - bore_od ≥ 4 mm (sub-CNC mill limit).
//   - slot_arc_deg in [10°, 120°] (extremes give no useful adjustment).
//   - slot_arc_deg ≤ 120° to keep at least one bridge per side.
//
// Signature: is_compound = true, subfeature_count = 4.
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace eccentric_bushing_seat {

constexpr const char* kSkillId = "eccentric_bushing_seat";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };
    double      bore_od_mm      = 0.0;            // bushing seat OD
    double      bore_depth_mm   = 0.0;
    double      slot_arc_deg    = 60.0;           // adjustment arc, each side
    double      slot_width_mm   = 1.6;            // tangential thickness of slot
    double      slot_depth_mm   = 0.0;            // axial depth of each slot
    double      chamfer_size_mm = 0.5;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace eccentric_bushing_seat
}  // namespace koocadcam::skill
