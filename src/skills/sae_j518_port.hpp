#pragma once
// @lat: [[engine/skills#sae_j518_port]]
//
// sae_j518_port — SAE J518 4-bolt split-flange hydraulic port (Code 61 style).
//
// COMPOUND of:
//   1. Central through / blind bore (port flow passage).
//   2. Four bolt-clearance holes on a PCD around the bore.
//   3. Captive face O-ring groove (annular groove around the central bore).
//   4. Chamfered entry on the central bore.
//
// All dimensions come from the central spec table `_hydraulic_ports_table.hpp`
// (kSaeJ518).  Only the size code (`"-08"/"-12"/"-16"/"-24"`) needs to be
// provided in the input; thru-bore, PCD, bolt dia, O-ring ID, O-ring CS are
// derived from the table.
//
// DFM gates:
//   DFM-INPUT      : non-positive / empty sizes.
//   DFM-J518-CODE  : sae_code not in {-08,-12,-16,-24}.
//   DFM-J518-DEPTH : face thickness < depth + 2 mm safety margin.
//   DFM-J518-XY    : position outside the workpiece XY bbox (less the flange
//                    radius margin).
//
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sae_j518_port {

constexpr const char* kSkillId = "sae_j518_port";

struct Input
{
    FaceDatum   face_id;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };
    std::string sae_code;                            // "-08" … "-24"
    double      depth_mm = 12.0;                     // port depth into part
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sae_j518_port
}  // namespace koocadcam::skill
