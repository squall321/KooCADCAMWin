#pragma once
// @lat: [[engine/skills#winch_drum_base_bolt_circle]]
//
// winch_drum_base_bolt_circle — Winch / windlass drum base (slice 16, marine /
// boat hardware).
//
// A sheet/halyard winch bolts to a deck pad on a circular bolt pattern.  The
// base prep machined into the deck pad / backing plate is:
//   1..N. bolt_count bolt holes on the base bolt-circle diameter, placed by
//         gp_Trsf::SetRotation — SEQUENTIAL cuts, no compound boolean.
//   N+1.  central shaft bore for the winch spindle.
//   N+2.  pawl pocket — a rectangular box recess for the self-tailing pawl /
//         ratchet stop.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1..bolt_count : bolt holes
//   then          : central shaft bore
//   then          : pawl pocket (box)
//
// subfeature_count = bolt_count + 2.
//
// DFM:
//   DFM-INPUT       : every dimension must be > 0.
//   DFM-MARINE-COUNT: bolt_count must be in [3, 8].
//   DFM-MARINE-SHAFT: bolt_circle must exceed shaft bore (bolts must clear it).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace winch_drum_base_bolt_circle {

constexpr const char* kSkillId = "winch_drum_base_bolt_circle";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy             { 0.0, 0.0, 0.0 };
    double      base_bolt_circle_dia_mm = 0.0;  // PCD of the base bolts
    int         bolt_count            = 0;        // 3..8 (marine winch range)
    double      bolt_dia_mm           = 0.0;      // bolt clearance hole dia
    double      shaft_bore_dia_mm     = 0.0;      // central spindle bore
    double      pawl_pocket_len_mm    = 0.0;
    double      pawl_pocket_wid_mm    = 0.0;
    double      pawl_pocket_depth_mm  = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace winch_drum_base_bolt_circle
}  // namespace koocadcam::skill
