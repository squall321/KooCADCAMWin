#pragma once
// @lat: [[engine/skills#pv_frame_clamp_tslot]]
//
// pv_frame_clamp_tslot — Mid/end clamp T-slot seat for a PV panel frame
// (slice 16, solar / PV mounting).
//
// Module mounting clamps engage the extruded aluminum panel frame via a
// T-slot channel.  Code-compliant grounding (UL 2703 / NEC 690.43) requires
// the clamp to "bite" the anodized frame: small grounding teeth penetrate
// the anodize layer to establish bonded continuity to the rail.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. T-slot channel (box cut DOWN from top face)
//   N. grounding teeth (small box notches in a row along the slot floor)
//
// subfeature_count = 1 + tooth_count.
//
// DFM:
//   DFM-INPUT  : every dimension must be > 0.
//   DFM-TEETH  : tooth_count in [1, 8].

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pv_frame_clamp_tslot {

constexpr const char* kSkillId = "pv_frame_clamp_tslot";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy      { 0.0, 0.0, 0.0 };
    double    slot_width_mm  = 10.0;
    double    slot_depth_mm  = 6.0;
    double    slot_length_mm = 40.0;
    int       tooth_count    = 4;
    double    tooth_depth_mm = 0.8;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pv_frame_clamp_tslot
}  // namespace koocadcam::skill
