#pragma once
// @lat: [[engine/skills#stringer_run_out_cutout]]
//
// stringer_run_out_cutout — Stringer run-out / mouse-hole cutout in a rib web
// (aerospace structures).
//
// Where a stiffening stringer passes through a transverse rib, the rib web is
// notched: a rectangular cutout clears the stringer cross-section, and a
// radiused "mouse hole" at the lower corner relieves the stress concentration
// at the web/stringer junction.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. rectangular stringer cutout (box)
//   2. radiused mouse hole (cylinder) at the cutout corner
//   subfeature_count = 2.
//
// DFM:
//   DFM-INPUT  : every dimension must be > 0.
//   DFM-RADIUS : mouse_hole_radius_mm must be < min(width, height)/2
//                (otherwise the relief swallows the whole cutout).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace stringer_run_out_cutout {

constexpr const char* kSkillId = "stringer_run_out_cutout";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy           { 0.0, 0.0, 0.0 };   // cutout centre (XY)
    double    stringer_width_mm   { 24.0 };
    double    stringer_height_mm  { 18.0 };
    double    mouse_hole_radius_mm{ 5.0 };
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace stringer_run_out_cutout
}  // namespace koocadcam::skill
