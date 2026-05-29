#pragma once
// @lat: [[engine/skills#check_valve_seat_with_stop]]
//
// check_valve_seat_with_stop — swing/lift check valve body internals.
//
// Sub-features (REAL cuts):
//   1. Inlet bore — cylindrical bore from upstream end.
//   2. Seat shoulder (45° conical) — connects inlet to a smaller through dia.
//      The disc rests on this conical shoulder when closed.
//   3. Outlet bore — downstream cylindrical bore (smaller than the seat
//      shoulder OD).
//   4. Disc-stop protrusion — a small interior boss (fused into body) that
//      limits upward disc travel.  Implemented as a "leave material" boss
//      by cutting a relief AROUND it (i.e. the stop is what survives).
//
// DFM (API 594 — Check Valves):
//   - Seat angle 30..60° (API 594 §6.3) — 45° is the standard.
//   - Disc-stop must NOT contact the seat at full-open (we assert stop
//      length ≤ inlet length − 4 mm).
//   - Inlet dia ≥ outlet dia (else flow is throttled at seat — bad for NRV).
//
// Recognize: 2 coaxial cylindrical faces of different radius + 1 conical
// face between them (the seat shoulder) + a small annular planar face
// (the stop protrusion's seating ring).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace check_valve_seat_with_stop {

constexpr const char* kSkillId = "check_valve_seat_with_stop";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm        = 0.0;          // inlet face center
    double      center_y_mm        = 0.0;
    double      center_z_mm        = 0.0;
    gp_Dir      flow_axis          { 1.0, 0.0, 0.0 };

    double      inlet_dia_mm       = 30.0;         // upstream cylindrical dia
    double      inlet_length_mm    = 22.0;         // length of inlet section
    double      seat_angle_deg     = 45.0;         // conical seat shoulder angle
    double      seat_drop_mm       = 6.0;          // radial drop seat→outlet
    double      outlet_dia_mm      = 18.0;         // downstream cylindrical dia
    double      outlet_length_mm   = 22.0;         // length of outlet section
    double      stop_dia_mm        = 8.0;          // disc-stop boss diameter
    double      stop_length_mm     = 6.0;          // disc-stop axial length
    double      stop_offset_mm     = 4.0;          // axial offset of stop INTO inlet
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace check_valve_seat_with_stop
}  // namespace koocadcam::skill
