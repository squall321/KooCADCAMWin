#pragma once
// @lat: [[engine/skills#needle_bearing_seat_press_fit]]
//
// needle_bearing_seat_press_fit — COMPOUND press-fit seat for a needle
// roller bearing (e.g. INA/Schaeffler HK series):
//
//   sub 1: precision bore at outer_dia (H7 fit class — tight running for
//          the press-fit OD of a needle cup)
//   sub 2: retention shoulder at retention_dia (axial backstop preventing
//          the bearing from walking out under load)
//   sub 3: 0.5 × 30° lead-in chamfer (standard SKF press-fit lead-in)
//
// All sub-features concentric on `axis_dir`.  The chamfer is fused with
// the bore tool before the boolean cut (single coherent cut).
//
// Press-fit standard (INA HK series application notes):
//   - H7 tolerance class is the assumed bore quality.
//   - lead-in chamfer 0.5 mm × 30° (chamfer_axial × chamfer_angle).  This
//     is fixed at the slice-9 level (no parameter exposed).
//   - retention_dia < outer_dia (smaller than the bore) so it acts as a
//     stop at the bore's depth.  Retention shoulder length = outer_dia / 8.
//
// DFM checks:
//   DFM-INPUT          : non-positive dims
//   DFM-SEAT-GEOM      : retention_dia ≥ outer_dia (cannot retain)
//   DFM-002            : retention_dia < 0.8 mm (sub-min)
//   DFM-BORE-RATIO     : depth/outer_dia > 4 (deep press-fit; deflection)
//   DFM-PRESS-FIT-MIN  : outer_dia < 3.0 mm (press-fit not practical
//                        on this OD — interference forces are unreliable)
//
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace needle_bearing_seat_press_fit {

constexpr const char* kSkillId = "needle_bearing_seat_press_fit";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };
    double      outer_dia_mm    = 0.0;
    double      depth_mm        = 0.0;
    double      retention_dia_mm = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace needle_bearing_seat_press_fit
}  // namespace koocadcam::skill
