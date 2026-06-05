#pragma once
// @lat: [[engine/skills#v_belt_pulley_groove]]
//
// v_belt_pulley_groove — V-belt sheave groove cut (classical A/B and SPZ
// wedge sections) — power transmission.
//
// A V-belt sheave is turned from a blank disc, then N circumferential
// V-grooves are cut into the outside diameter.  Each groove is a solid of
// revolution with a trapezoidal (V-wedge) cross-section whose included
// angle is groove_angle_deg (34–38° per ISO 4183 / RMA IP-20).  The groove
// flanks grip the belt sides.
//
// Each groove cutter is modeled as a single wedge ring (two cone frusta
// meeting at the groove root plane, fused — non-overlapping along Z), then
// the N grooves are cut SEQUENTIALLY at stacked axial bands on the OD.
//
// Sub-features (SEQUENTIAL pr::cut, no overlapping-cutter compound boolean):
//   1..N. N V-grooves removed from the outside diameter.
//
// subfeature_count = groove_count.
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-PT-SECTION: belt_section in { "A", "B", "SPZ" }.
//   DFM-PT-ANGLE  : groove_angle_deg in [34, 38].
//   DFM-PT-WIDTH  : groove_count grooves + lands must fit in the OD face.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace v_belt_pulley_groove {

constexpr const char* kSkillId = "v_belt_pulley_groove";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy        { 0.0, 0.0, 0.0 };
    double      pulley_od_mm     = 0.0;     // sheave outside diameter
    std::string belt_section;               // "A" | "B" | "SPZ"
    int         groove_count     = 0;       // number of V-grooves
    double      groove_angle_deg = 0.0;     // included angle 34–38
    double      groove_depth_mm  = 0.0;     // radial depth of each groove
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace v_belt_pulley_groove
}  // namespace koocadcam::skill
