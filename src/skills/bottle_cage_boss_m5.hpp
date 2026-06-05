#pragma once
// @lat: [[engine/skills#bottle_cage_boss_m5]]
//
// bottle_cage_boss_m5 — Water-bottle cage boss pair (M5, 64 mm spacing).
//
// Every bicycle frame carries the standard ISO 4210 bottle-cage interface:
// two M5 threaded bosses spaced 64 mm apart.  On thin-wall frames the bosses
// are rivet-nuts (riv-nuts), so each location is a counterbored seat that
// receives the riv-nut flange plus a tapped pilot for the M5 thread.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean — the two coaxial
// sections at each boss are STACKED along Z, not overlapping):
//   1. riv-nut seat counterbore at boss A (wide, shallow)
//   2. riv-nut seat counterbore at boss B
//   3. tapped M5 pilot at boss A (narrow, deep — below the seat)
//   4. tapped M5 pilot at boss B
//
// subfeature_count = 4.  Material is REMOVED (derived_volume_removed_mm3).
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-SPACING   : boss_spacing_mm must be the 64 mm ISO 4210 standard.
//   DFM-THREAD    : thread_key must exist in the central ISO M-thread table.
//   DFM-SEAT      : rivnut_seat_dia_mm must exceed the M5 pilot diameter.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bottle_cage_boss_m5 {

constexpr const char* kSkillId = "bottle_cage_boss_m5";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      row_origin       { 0.0, 0.0, 0.0 };  // boss A center (top face)
    double      boss_spacing_mm  = 64.0;             // ISO 4210 spacing
    std::string thread_key       = "M5";            // central ISO M-thread key
    double      rivnut_seat_dia_mm = 8.0;            // riv-nut flange seat dia
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bottle_cage_boss_m5
}  // namespace koocadcam::skill
