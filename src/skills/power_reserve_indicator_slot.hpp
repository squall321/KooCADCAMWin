#pragma once
// @lat: [[engine/skills#power_reserve_indicator_slot]]
//
// power_reserve_indicator_slot — Arc-shaped power-reserve indicator window
// in a watch dial plate (slice 16, watch advanced complications).
//
// A power-reserve complication shows remaining mainspring wind via a hand
// sweeping an arc.  The dial-plate machining is:
//   - an arc-shaped indicator window slot, approximated by N small radial
//     box cuts swept along the arc (each rotated about the dial centre)
//   - a hand-clearance pocket beneath the arc so the indicator hand can sweep
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1..N. arc segment box cuts (arc_segment_count of them)
//   N+1.  hand-clearance pocket (annular ring under the arc)
//
// subfeature_count = arc_segment_count + 1.
//
// DFM:
//   DFM-INPUT     : arc_radius/slot_width/slot_depth must be > 0.
//   DFM-ARC       : arc_end_deg must exceed arc_start_deg.
//   DFM-SEGMENTS  : arc_segment_count must be in [4, 48].
//   DFM-DEPTH     : slot_depth_mm must be < stock thickness.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace power_reserve_indicator_slot {

constexpr const char* kSkillId = "power_reserve_indicator_slot";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy         { 0.0, 0.0, 0.0 };   // dial / arc centre
    double      arc_radius_mm      = 8.0;
    double      arc_start_deg      = 30.0;
    double      arc_end_deg        = 150.0;
    int         arc_segment_count  = 12;
    double      slot_width_mm       = 1.2;
    double      slot_depth_mm       = 0.8;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace power_reserve_indicator_slot
}  // namespace koocadcam::skill
