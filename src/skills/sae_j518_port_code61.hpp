#pragma once
// @lat: [[engine/skills#sae_j518_port_code61]]
//
// sae_j518_port_code61 — SAE J518 Code 61 (3000 psi) 4-bolt split-flange
// hydraulic port.  Compound macro covering the full mobile-hydraulics
// dash range -8 / -12 / -16 / -20 / -24 / -32 with metric M-bolt clearance
// holes.
//
// COMPOUND of:
//   1. Central port through-bore (sized for tube OD clearance).
//   2. Captive face O-ring groove (BSE flat groove on the flange face).
//   3. Four M-bolt clearance holes on the table-specified PCD.
//
// All dimensions come from the central spec table `_hydraulic_ports.hpp`
// (kSaeJ518Code61).
//
// DFM gates:
//   DFM-INPUT       : depth ≤ 0.
//   DFM-J518C61-CODE: dash_size not in table.
//   DFM-J518C61-FACE: face_id does not resolve to a planar face.
//   DFM-J518C61-MAT : insufficient material around bore
//                     (face XY span < 1.5 × bolt_circle_dia).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sae_j518_port_code61 {

constexpr const char* kSkillId = "sae_j518_port_code61";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm  = 0.0;
    double      center_y_mm  = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };
    std::string dash_size;                          // "-8" .. "-32"
    double      depth_mm = 14.0;                    // port bore depth
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sae_j518_port_code61
}  // namespace koocadcam::skill
