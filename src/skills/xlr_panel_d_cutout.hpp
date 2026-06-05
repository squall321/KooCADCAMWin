#pragma once
// @lat: [[engine/skills#xlr_panel_d_cutout]]
//
// xlr_panel_d_cutout — Neutrik D-series XLR panel cutout (slice 16, pro
// audio hardware).
//
// The Neutrik D-series chassis connector mounts in a panel via a 24.0 mm
// main bore plus two 3.2 mm (#4 / M3) mounting screw holes on a horizontal
// centre line at 19.05 mm (0.750 inch) spacing.  This skill machines that
// standardized cutout from a flat panel (top) face:
//   1. main connector bore (24.0 mm through)
//   2..3. two mounting screw holes at +/- (spacing/2) along X
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. main bore           (cylinder cut through)
//   2..3. two mount holes  (cylinders cut through)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT  : mount hole dia and spacing must be > 0.
//   DFM-BORE   : bore_dia must be within [20, 30] mm (Neutrik D footprint).
//   DFM-MOUNT  : mount holes must clear the main bore wall.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace xlr_panel_d_cutout {

constexpr const char* kSkillId = "xlr_panel_d_cutout";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy        { 0.0, 0.0, 0.0 };
    double    bore_dia_mm      { 24.0 };    // Neutrik D main bore
    double    mount_hole_dia_mm{ 3.2 };     // #4 / M3 mounting screws
    double    mount_spacing_mm { 19.05 };   // 0.750 inch
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace xlr_panel_d_cutout
}  // namespace koocadcam::skill
