#pragma once
// @lat: [[engine/skills#brass_valve_port_compound]]
//
// brass_valve_port_compound — brass instrument (trumpet / trombone) valve port.
//
// Slice 16 — audio / musical instrument hardware.
//
// Sub-features (sequential CUTs):
//   1. central valve bore (cylinder cut, valve_bore_dia × stock thickness)
//   2..(1+N). N angular slide-tube ports arranged radially around the valve
//             axis at `slide_port_radial_distance_mm` from centre.  Each port
//             is a horizontal cylinder cut from the outside surface inward.
//
// For a trumpet (3 valves) slide_port_count = 3 typical; pitch typ 120 deg.
//
// subfeature_count = 1 + slide_port_count.
//
// DFM gates:
//   DFM-INPUT       : positive dims
//   DFM-PORT-COUNT  : slide_port_count in [2, 6]
//   DFM-PITCH       : slide_port_pitch_deg in (0, 180]
//   DFM-RADIUS      : slide ports must clear the central bore (radial_distance
//                     - slide_port_dia/2 > valve_bore/2)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace brass_valve_port_compound {

constexpr const char* kSkillId = "brass_valve_port_compound";

struct Input
{
    FaceDatum   face_id;
    double      center_x_mm                  = 0.0;
    double      center_y_mm                  = 0.0;
    gp_Dir      valve_axis_dir               { 0.0, 0.0, 1.0 };
    double      valve_bore_dia_mm            = 13.0;   // typ 12-14 mm
    int         slide_port_count             = 3;      // typ 3 for trumpet
    double      slide_port_dia_mm            = 11.0;
    double      slide_port_pitch_deg         = 120.0;  // angular spacing
    double      slide_port_radial_distance_mm = 15.0;  // radial offset
    double      slide_port_depth_mm          = 12.0;   // depth radial in
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace brass_valve_port_compound
}  // namespace koocadcam::skill
