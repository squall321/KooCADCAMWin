#pragma once
// @lat: [[engine/skills#anti_pid_coating_recess]]
//
// anti_pid_coating_recess — SOLAR/PV compound: cell-carrier substrate
// recess array sized to host an anti-PID (Potential Induced Degradation)
// coating layer at each cell location.
//
// Sub-feature chain (SEQUENTIAL pr::cut, N × M ops):
//   For each (ix, iy) in cell_array_count_x × cell_array_count_y:
//     CUT shallow rectangular recess of (cell_pitch_x - 2×margin) ×
//                                       (cell_pitch_y - 2×margin) ×
//                                       recess_depth
//     centred on (ix × cell_pitch_x, iy × cell_pitch_y).
//
// Signature:
//   pattern.kind                = "anti_pid_coating_recess"
//   pattern.is_compound         = true
//   pattern.solar_feature_type  = "anti_pid_recess_array"
//   pattern.subfeature_count    = N × M
//   pattern.cell_count
//   pattern.recess_depth_mm
//
// DFM gates:
//   - all positive
//   - recess_depth in [0.1, 1.5] mm (anti-PID coating typical range)
//   - cell array reasonable: counts in [1, 24] each, pitch >= 2 mm
//   - coating_margin > 0 and < min(cell_pitch_x, cell_pitch_y) / 2

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace anti_pid_coating_recess {

constexpr const char* kSkillId = "anti_pid_coating_recess";

struct Input
{
    int    face_id             = -1;
    double origin_x_mm         = 0.0;
    double origin_y_mm         = 0.0;
    int    cell_array_count_x  = 2;
    int    cell_array_count_y  = 2;
    double cell_pitch_x_mm     = 12.0;
    double cell_pitch_y_mm     = 12.0;
    double recess_depth_mm     = 0.5;  // anti-PID coating layer typical
    double coating_margin_mm   = 0.8;  // margin between cell recess edges
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace anti_pid_coating_recess
}  // namespace koocadcam::skill
