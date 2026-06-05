#pragma once
// @lat: [[engine/skills#cutless_bearing_housing_seat]]
//
// cutless_bearing_housing_seat — Cutless (cutlass) bearing housing seat
// (slice 16, marine / boat hardware).
//
// A cutless bearing is the water-lubricated rubber-and-shell sleeve the
// propeller shaft runs in, pressed into a strut or stern-tube housing.  The
// housing prep is:
//   1. bearing housing bore — sized to the bearing OD as a press fit using
//      the ISO 286 P7 hole limit so the bearing shell stays captive.
//   2 & 3. two set-screw clearance holes drilled radially to lock the
//          bearing against rotation; clearance from the central metric table.
//   then  : groove_count internal water-relief grooves (annularRing) that let
//           cooling/lubricating water flow along the shaft.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. bearing housing bore (P7 sized)
//   2. set-screw clearance hole #1
//   3. set-screw clearance hole #2
//   then groove_count water-relief grooves
//
// subfeature_count = 1 + 2 + groove_count.
//
// DFM:
//   DFM-INPUT       : bearing_od_mm and housing_length_mm must be > 0.
//   DFM-M-THREAD    : set_screw_thread_key must exist in central metric table.
//   DFM-MARINE-GROOVE : groove_count must be in [1, 6].

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cutless_bearing_housing_seat {

constexpr const char* kSkillId = "cutless_bearing_housing_seat";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      axis_origin        { 0.0, 0.0, 0.0 };  // XY of the bore axis
    double      bearing_od_mm      = 0.0;   // bearing shell OD (drives P7 bore)
    double      housing_length_mm  = 0.0;   // axial length of the housing bore
    std::string set_screw_thread_key;       // from _iso_thread_table.hpp ("M6"…)
    int         groove_count       = 0;     // number of water-relief grooves
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cutless_bearing_housing_seat
}  // namespace koocadcam::skill
