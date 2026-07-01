#pragma once
// @lat: [[engine/skills#crown_knurl]]
//
// crown_knurl — a ring of N radial notches around a protruding cone knob (a
// watch crown's grip knurl).  Each notch is a small cylinder cut RADIALLY into
// the knob rim (its axis perpendicular to the knob axis), spaced evenly at
// 2*pi/N.  This is a subtractive, geometric feature (real material removal),
// distinct from the lathe/turning `knurl` skill (which only displaces material
// and has a metadata-only recognizer).
//
//   apply():  build N notch cutters around `world_center` + `world_axis` at
//             radius `knob_radius_mm`, cut them all into the workpiece.
//
// Recognise (geometric, foreign): a cone knob (anchor) surrounded by N small,
// equal-radius cylinders whose axes are PERPENDICULAR to the knob axis and whose
// centres lie on a circle of radius ~= knob_radius, evenly angularly spaced.
// The crown SHAFT (a single axial bore, axis PARALLEL to the knob) and the side
// buttons (non-uniform) are rejected by the perpendicular-axis + even-spacing +
// count>=3 gates.
//
// DFM:
//   - notch_radius_mm > 0, knob_radius_mm > 0, count >= 3

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace crown_knurl {

constexpr const char* kSkillId = "crown_knurl";

struct Input
{
    // World placement (a knurl sits on a curved/protruding knob — no planar datum).
    double world_cx_mm = 0.0, world_cy_mm = 0.0, world_cz_mm = 0.0;  // knob-rim ring centre
    double world_ax = 0.0, world_ay = 0.0, world_az = 1.0;          // knob axis (outward)
    double knob_radius_mm  = 0.0;    // radius of the ring the notches sit on
    double notch_radius_mm = 0.0;    // radius of each notch cutter cylinder
    int    count           = 0;      // number of notches (>= 3)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace crown_knurl
}  // namespace koocadcam::skill
