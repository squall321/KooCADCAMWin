#pragma once
// @lat: [[engine/skills#rudder_gudgeon_pintle_bore]]
//
// rudder_gudgeon_pintle_bore — Rudder gudgeon pintle bore (slice 16, marine /
// boat hardware).
//
// A rudder hangs on gudgeon-and-pintle hinges.  The gudgeon (the female half
// bolted to the transom) carries a bushing pressed into a bore; the pintle
// (the male pin on the rudder) swings inside that bushing.  The bore prep is:
//   1. pintle bushing bore — sized to the bushing OD as a press fit; the
//      finished bore uses the ISO 286 H7 upper limit so the bushing presses
//      in with the correct interference.
//   2. grease groove — an internal annular groove (annularRing) part-way down
//      the bore that retains grease around the pintle.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. pintle bushing bore (H7 sized)
//   2. grease groove (annular ring)
//
// subfeature_count = 2.
//
// DFM:
//   DFM-INPUT      : every dimension must be > 0.
//   DFM-MARINE-FIT : bushing_od_mm must exceed pintle_dia_mm (the bushing wall
//                    must exist) and bore_depth must clear the groove.
//   DFM-MARINE-GROOVE : groove_width_mm must be < bore_depth_mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rudder_gudgeon_pintle_bore {

constexpr const char* kSkillId = "rudder_gudgeon_pintle_bore";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy        { 0.0, 0.0, 0.0 };
    double      pintle_dia_mm    = 0.0;   // the swinging pintle pin dia
    double      bushing_od_mm    = 0.0;   // bushing OD (drives the H7 bore)
    double      groove_width_mm  = 0.0;   // internal grease groove width
    double      bore_depth_mm    = 0.0;   // depth of the bushing bore
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rudder_gudgeon_pintle_bore
}  // namespace koocadcam::skill
