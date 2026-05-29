#pragma once
// @lat: [[engine/skills#lifting_lug_pad_eye]]
//
// lifting_lug_pad_eye — COMPOUND structural skill that builds a padeye
// (lifting-lug) plate: rectangular plate body with a thru hole for a
// shackle pin, an entry-side chamfer around the hole, and a base weld-prep
// cut where the lug welds to its host structure.
//
//   sub-feature 1: plate body (lug_width × lug_height × plate_t)
//   sub-feature 2: shackle pin hole (thru cylinder, dia = pin_hole_dia)
//   sub-feature 3: 45° face chamfer around the pin hole (entry side)
//   sub-feature 4: base weld-prep cut (45° bevel at lug base for fillet weld)
//
// subfeature_count = 4.
//
// Plate lies in the XZ plane (z-axis is height-up; x-axis is width).
// Base is at z = 0, top at z = lug_height.  Plate thickness extends -t/2 to +t/2 in Y.
//
// DFM (DNV-OS-H205 + AISC ASCE 7 lifting-lug rules):
//   DFM-LUG-INPUT      : any non-positive input
//   DFM-LUG-PIN-EDGE   : (lug_height - pin_z) < 1.5 × pin_dia  (insufficient
//                        edge distance above the pin per DNV)
//   DFM-LUG-AR         : lug_width / lug_height outside [0.6, 1.5]
//   DFM-LUG-T-PIN      : plate_t < pin_dia × 0.5

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lifting_lug_pad_eye {

constexpr const char* kSkillId = "lifting_lug_pad_eye";

struct Input
{
    double plate_t_mm       = 25.0;
    double pin_hole_dia_mm  = 40.0;
    double lug_height_mm    = 200.0;
    double lug_width_mm     = 150.0;
    double chamfer_mm       = 3.0;     // 45° entry chamfer around pin hole
    double base_bevel_mm    = 8.0;     // base weld-prep bevel size
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lifting_lug_pad_eye
}  // namespace koocadcam::skill
