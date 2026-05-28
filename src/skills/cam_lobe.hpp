#pragma once
// @lat: [[engine/skills#cam_lobe]]
//
// cam_lobe — synthesize a SINGLE offset lobe on a cylindrical workpiece by
// SUBTRACTING the cylinder OUTSIDE the lobe (i.e. cutting everything down
// to base_radius_mm except a finite angular sector that is left at
// base_radius + lobe_lift_mm).  This produces a profile with one bump.
//
// Strategy: build a smaller cylinder of radius = base_radius_mm; then
// fuse back a wedge sector representing the lobe (between base_radius
// and base_radius + lobe_lift) covering `lobe_angle_deg` of arc.
//
//          (cross section)
//                    ╱── lobe bump (lobe_angle_deg wide, lift = +lobe_lift_mm)
//                   ▼
//              ┌────┐
//          ╭───┘    └───╮
//        ╭┘              ╰╮
//        │  ◯ axis        │   ← base_radius_mm (the rest of the cam profile)
//        ╰╮              ╭╯
//          ╰────────────╯
//
// Signature pattern:
//   - cylindrical face (base)
//   - 1 raised bump (lobe sector)
//
// DFM checks:
//   DFM-CAM-LIFT  : lobe_lift_mm > 0 and < 10 × base_radius
//   DFM-CAM-ANG   : lobe_angle_deg in (0, 359.9]
//   DFM-CAM-RAD   : base_radius_mm > 0.5

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cam_lobe {

constexpr const char* kSkillId = "cam_lobe";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm       = 0.0;     // cam axis x
    double      center_y_mm       = 0.0;     // cam axis y
    double      angle_deg         = 0.0;     // center angle of the lobe bump (around +Z)
    double      lobe_lift_mm      = 1.0;     // radial bump above base_radius
    double      lobe_angle_deg    = 30.0;    // arc width of the lobe bump
    double      base_radius_mm    = 10.0;    // cam base circle radius
    double      thickness_mm      = 5.0;     // axial extent (along cam axis)
    gp_Dir      axis_dir          { 0.0, 0.0, 1.0 };  // cam rotation axis
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cam_lobe
}  // namespace koocadcam::skill
