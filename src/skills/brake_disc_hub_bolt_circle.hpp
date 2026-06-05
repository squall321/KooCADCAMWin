#pragma once
// @lat: [[engine/skills#brake_disc_hub_bolt_circle]]
//
// brake_disc_hub_bolt_circle — Rolling-stock brake-disc mounting hub (slice 16,
// railway / rolling stock).
//
// A railway brake disc bolts to the wheel/axle hub through a bolt circle and
// is ventilated by radial slots.  The hub face carries:
//   1. central hub bore — locates the disc on the axle.
//   2. N-bolt mounting circle — bolt holes on a PCD, indexed by SetRotation
//      and cut SEQUENTIALLY (never as one overlapping compound boolean).
//   3. radial ventilation slots — rectangular slots cut radially around the
//      disc, also indexed by SetRotation and cut sequentially.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. central hub bore (cylinder, through)
//   2..(1+bolt_count)            N bolt holes (rotated cylinders)
//   (2+bolt_count)..end          N vent slots (rotated boxes)
//
// subfeature_count = 1 + bolt_count + vent_slot_count.
//
// DFM:
//   DFM-INPUT  : every dimension must be > 0.
//   DFM-COUNT  : bolt_count in [4, 12].
//   DFM-PCD    : bolt_circle_dia_mm must exceed hub_bore_dia_mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace brake_disc_hub_bolt_circle {

constexpr const char* kSkillId = "brake_disc_hub_bolt_circle";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy           { 0.0, 0.0, 0.0 };
    double    hub_bore_dia_mm     = 80.0;    // central axle bore Ø
    double    bolt_circle_dia_mm  = 160.0;   // PCD
    int       bolt_count          = 6;       // [4, 12]
    double    bolt_dia_mm         = 14.0;
    int       vent_slot_count     = 8;
    double    vent_slot_width_mm  = 8.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace brake_disc_hub_bolt_circle
}  // namespace koocadcam::skill
