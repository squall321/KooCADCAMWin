#pragma once
// @lat: [[engine/skills#optical_axis_alignment_pin_pair]]
//
// optical_axis_alignment_pin_pair — OPTICAL compound: two precision dowel-pin
// holes for optical-bench / bracket alignment.
//
// Sub-features (2 ops):
//   1. cut precision pin hole A at (pin_a_xy)  (CUT)
//   2. cut precision pin hole B at (pin_b_xy)  (CUT)
//
// ISO 286 H6/H7 fit class applied (see _iso286_fits.hpp).
//
// DFM:
//   DFM-INPUT      : face_id valid, depth > 0
//   DFM-PIN-DIA    : pin_dia_mm must lie inside H6/H7 lookup band (0 < d ≤ 100)
//   DFM-PIN-TOL    : tolerance_class ∈ {H6, H7}
//   DFM-PIN-SPACING: two pin centres must be at least 2×pin_dia apart

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>

namespace koocadcam::skill {

class Workpiece;

namespace optical_axis_alignment_pin_pair {

constexpr const char* kSkillId = "optical_axis_alignment_pin_pair";

struct Input
{
    int         face_id          = -1;
    double      pin_a_x_mm       = 0.0;
    double      pin_a_y_mm       = 0.0;
    double      pin_b_x_mm       = 0.0;
    double      pin_b_y_mm       = 0.0;
    double      pin_dia_mm       = 4.0;
    double      pin_depth_mm     = 8.0;
    std::string tolerance_class  = "H7";   // "H6" | "H7"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace optical_axis_alignment_pin_pair
}  // namespace koocadcam::skill
