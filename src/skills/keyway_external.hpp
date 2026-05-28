#pragma once
// @lat: [[engine/skills#keyway_external]]
//
// keyway_external — cut a rectangular axial slot on the OUTER cylindrical
// surface of a shaft (counterpart to the internal `mill_keyway`, which
// operates on a flat surface).  The slot is parallel to the shaft axis and
// removes material radially inward.
//
//        (looking along shaft axis)
//                     ┌──┐  ← keyway slot (depth_mm radial, width_mm along arc)
//                ┌────┘  └────┐
//             ╔══╝            ╚══╗
//            ╔╝                  ╚╗
//            ║         ◯  axis    ║   shaft OD
//            ╚╗                  ╔╝
//             ╚══╗            ╔══╝
//                └────────────┘
//
// The cut is approximated as an axis-aligned rectangular box whose tangent
// extent is `width_mm` (chord across the OD at the slot bottom) and whose
// radial extent is `depth_mm` from OD inward.
//
// Signature pattern:
//   - 4 new planar wall faces (2 axial-end walls + 2 long side walls)
//   - 1 new planar floor face (radial bottom of the slot)
//
// DFM checks:
//   DFM-002       : width_mm ≥ 0.8 mm
//   DFM-KEY-DEPTH : depth_mm < shaft_radius_mm
//   length_mm > 0, depth_mm > 0

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace keyway_external {

constexpr const char* kSkillId = "keyway_external";

struct Input
{
    FaceDatum   entry_face;
    // Shaft axis (typically world Z).
    gp_Dir      shaft_axis        { 0.0, 0.0, 1.0 };
    double      shaft_center_x_mm = 0.0;
    double      shaft_center_y_mm = 0.0;
    double      shaft_radius_mm   = 5.0;
    // Slot placement: angular position on the shaft (radians around shaft_axis),
    // axial center along shaft_axis, axial length, tangent width, radial depth.
    double      angle_rad         = 0.0;
    double      axial_center_mm   = 0.0;
    double      length_mm         = 0.0;    // axial extent
    double      width_mm          = 0.0;    // tangent (chord) extent
    double      depth_mm          = 0.0;    // radial extent (into shaft)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace keyway_external
}  // namespace koocadcam::skill
