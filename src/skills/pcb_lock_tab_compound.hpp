#pragma once
// @lat: [[engine/skills#pcb_lock_tab_compound]]
//
// pcb_lock_tab_compound — EV BATTERY compound: sheet-metal lock tab for
// retaining a BMS PCB on the housing.
//
// Sub-feature chain (2 ops):
//   1. FUSE a small rectangular sheet-metal tab on the face
//      (tab_width × tab_length × tab_height_above_face).  Tab is oriented
//      with its long axis along +X (caller may rotate by reframing the
//      input face).
//   2. CUT a narrow engagement slot UNDER the tab on the face plane
//      (slot_engagement_depth deep, slot is tab_width × 1.2 mm = a
//      flat horizontal undercut for the PCB edge to slide into).
//
// Signature:
//   pattern.kind             = "pcb_lock_tab_compound"
//   pattern.is_compound      = true
//   pattern.ev_feature_type  = "pcb_lock_tab"
//   pattern.subfeature_count = 2
//   pattern.derived_volume_added_mm3
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   - all positive
//   - tab_width ∈ [3, 5] mm typ (warn outside)
//   - tab_length ∈ [5, 10] mm typ (warn outside)
//   - tab_height_above_face ≥ 0.5 mm (formable)
//   - tab_height_above_face ≤ 4 mm (otherwise tab is a boss, not a sheet
//     tab)
//   - slot_engagement_depth ≥ 0.4 mm (DFM-MIN-WALL: enough material left)
//   - slot_engagement_depth < tab_height_above_face (undercut must be
//     bounded by tab height)

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pcb_lock_tab_compound {

constexpr const char* kSkillId = "pcb_lock_tab_compound";

struct Input
{
    int    face_id                   = -1;
    double tab_position_x_mm         = 0.0;
    double tab_position_y_mm         = 0.0;
    double tab_width_mm              = 4.0;   // 3-5 typ
    double tab_length_mm             = 6.0;   // 5-10 typ
    double tab_height_above_face_mm  = 1.5;   // 1-2 typ
    double slot_engagement_depth_mm  = 0.8;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pcb_lock_tab_compound
}  // namespace koocadcam::skill
