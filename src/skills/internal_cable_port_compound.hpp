#pragma once
// @lat: [[engine/skills#internal_cable_port_compound]]
//
// internal_cable_port_compound — Internal cable-routing entry/exit port.
//
// Modern frames route brake/derailleur/dropper cables internally.  Each
// entry/exit point is an angled bore through the frame wall plus a shallow
// annular groove around the mouth that retains a rubber grommet/port cover.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean — the angled bore is
// cut FIRST as the big feature, then the small grommet groove):
//   1. angled cable port bore (cylinder tilted by entry_angle_deg from
//      vertical, boring through the wall)
//   2. grommet retention groove (annular ring around the port mouth)
//
// subfeature_count = 2.
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-ANGLE     : entry_angle_deg in [0, 60] (steep ports tear out).
//   DFM-GROOVE    : grommet_groove_dia_mm must exceed port_dia_mm (ring must
//                   surround the bore mouth).
//   DFM-STOCK     : grommet groove must fit inside the stock.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace internal_cable_port_compound {

constexpr const char* kSkillId = "internal_cable_port_compound";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy             { 0.0, 0.0, 0.0 };  // port mouth XY (top face)
    double    port_dia_mm           = 6.0;              // cable port bore
    double    grommet_groove_dia_mm = 10.0;             // grommet groove OD
    double    grommet_groove_depth_mm = 1.2;            // groove depth
    double    entry_angle_deg       = 30.0;             // tilt from vertical
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace internal_cable_port_compound
}  // namespace koocadcam::skill
