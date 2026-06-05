#pragma once
// @lat: [[engine/skills#busbar_terminal_lug_compound]]
//
// busbar_terminal_lug_compound — SOLAR/PV compound: high-current copper
// busbar terminal lug with multi-bolt clamp pattern on a combiner-box or
// inverter chassis face.
//
// Sub-feature chain (SEQUENTIAL pr::cut, 1 + N ops):
//   1. CUT rectangular busbar slot at the face (busbar_w × busbar_thickness)
//   2..N+1. CUT lug_terminal_count tapped terminal bolt holes around the
//      lug centerline at radius = terminal_pcd/2.
//
// Signature:
//   pattern.kind                = "busbar_terminal_lug_compound"
//   pattern.is_compound         = true
//   pattern.solar_feature_type  = "busbar_terminal_lug"
//   pattern.subfeature_count    = 1 + lug_terminal_count
//   pattern.terminal_dia_mm
//   pattern.terminal_pcd_mm
//
// DFM gates:
//   - all positive
//   - terminal_dia_mm in standard PV-busbar set {8, 10, 12} mm
//   - lug_terminal_count in {1, 2, 4}
//   - busbar_width >= terminal_pcd + 2 × terminal_dia (so PCD fits)
//   - busbar_thickness in [3, 12] mm (current-carrying minimum)

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace busbar_terminal_lug_compound {

constexpr const char* kSkillId = "busbar_terminal_lug_compound";

struct Input
{
    int    face_id             = -1;
    double center_x_mm         = 0.0;
    double center_y_mm         = 0.0;
    double busbar_width_mm     = 25.0;
    double busbar_thickness_mm = 5.0;
    int    lug_terminal_count  = 2;     // 1 / 2 / 4
    double terminal_dia_mm     = 10.0;  // M8 / M10 / M12
    double terminal_pcd_mm     = 16.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace busbar_terminal_lug_compound
}  // namespace koocadcam::skill
