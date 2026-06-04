#pragma once
// @lat: [[engine/skills#cell_clamp_slot_array]]
//
// cell_clamp_slot_array — EV BATTERY compound: cylindrical cell-clamping
// pocket array.  Used in 18650 / 21700 modules where each cell sits in a
// recess and 4 radial clamp slots provide axial spring engagement.
//
// Sub-feature chain (per cell, cell_count = nx × ny):
//   1. CUT cylindrical pocket of cell_dia at each (i, j) position
//   2. CUT 4 axial clamp slots (N/S/E/W of the cell axis), slot_width wide,
//      slot_depth deep, extending radially outward by an extra slot_width.
//
// Signature:
//   pattern.kind                   = "cell_clamp_slot_array"
//   pattern.is_compound            = true
//   pattern.ev_feature_type        = "cell_clamp_array"
//   pattern.cell_count             = nx * ny
//   pattern.subfeature_count       = cell_count * 5  (1 pocket + 4 slots)
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   - all positive dims
//   - cell_dia ∈ [10, 30] mm (18650 = 18.0, 21700 = 21.0)
//   - cell pitch ≥ cell_dia + 0.5 (clearance between cells)
//   - clamp_slot_width typ 1.5 mm, ≥ 0.5
//   - clamp_slot_depth > 0 and < housing thickness (warn only)

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cell_clamp_slot_array {

constexpr const char* kSkillId = "cell_clamp_slot_array";

struct Input
{
    int    face_id              = -1;
    double origin_x_mm          = 0.0;
    double origin_y_mm          = 0.0;
    double cell_dia_mm          = 18.0;       // 18650 or 21.0 for 21700
    int    cell_count_x         = 4;
    int    cell_count_y         = 4;
    double cell_pitch_x_mm      = 20.0;
    double cell_pitch_y_mm      = 20.0;
    double clamp_slot_width_mm  = 1.5;
    double clamp_slot_depth_mm  = 6.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cell_clamp_slot_array
}  // namespace koocadcam::skill
