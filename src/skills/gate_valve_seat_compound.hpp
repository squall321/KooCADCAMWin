#pragma once
// @lat: [[engine/skills#gate_valve_seat_compound]]
//
// gate_valve_seat_compound — gate-valve internal cavity compound feature.
//
//   Layout (axis = horizontal X, gate stem = vertical -Z):
//
//        ┌─────── stem bore (top, axial -Z) ──────┐
//        │                                        │
//   ──┐  │           ╱──── gate slot ────╲       │  ┌──
//     │  ▼          ╱ rectangular through ╲     ▼  │
//     │ ===========(  cavity, perp. to flow )========│
//     │  ▲          ╲                      ╱     ▲  │
//     │  │ conical seat (left)   conical seat (rt) │
//     │  │                                        │
//   ──┘                                            └──
//                  bottom recess (sink for gate)
//
// Sub-features (all REAL geometric cuts):
//   1. Gate slot — rectangular box, perpendicular to flow axis, FULL through Y.
//   2. Left conical seat — cone frustum, narrow side facing the gate slot.
//   3. Right conical seat — symmetric cone frustum on the opposite side.
//   4. Stem bore — cylindrical hole from top (Z+) down to gate slot.
//   5. Bottom recess — small cylinder below the gate slot to accept the
//      retracted gate disc (so the disc fully clears the flow path when open).
//
// Volume removed (rough analytic): slot box + 2×cone − cone/slot overlap
//   + stem cylinder + bottom recess cylinder.
//
// DFM (DIN EN 1984 — Industrial valves, steel gate valves):
//   - Body wall thickness implied by seat OD vs stock OD: stock_OD − seat_OD
//     ≥ DN×0.05 + 5 mm (DIN EN 1984 Table 6 minimums).
//   - Seat conical half-angle 5..25° (industry — keeps seating force in line
//     with stem axial load; ASME B16.34 §6.4 trim guidance).
//   - Stem bore concentricity must align with stock central axis (intrinsic
//     here — taken as parameter).
//
// Recognize: geometric heuristic — search for a planar pair of vertical
// "gate slot" walls (faces with normal ⟂ flow axis and ⟂ stem axis) PLUS
// two conical faces sharing the flow axis PLUS a top stem cylinder.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gate_valve_seat_compound {

constexpr const char* kSkillId = "gate_valve_seat_compound";

struct Input
{
    FaceDatum   entry_face;                          // top face for stem bore (usually +Z)
    double      center_x_mm        = 0.0;            // gate slot center along flow axis
    double      center_y_mm        = 0.0;            // gate slot center, perp horizontal
    double      center_z_mm        = 0.0;            // body centerline along Z (flow Z)
    gp_Dir      flow_axis          { 1.0, 0.0, 0.0 };// flow direction (the bore axis)
    gp_Dir      stem_axis          { 0.0, 0.0, 1.0 };// stem direction (perp to flow)
    double      seat_bore_dia_mm   = 30.0;           // ID of flow path through seats
    double      seat_od_mm         = 40.0;           // OD of conical seat ring
    double      seat_taper_deg     = 15.0;           // half-angle of conical seat (5..25°)
    double      seat_length_mm     = 8.0;            // axial length of each seat ring
    double      slot_width_mm      = 12.0;           // gate slot opening, along flow axis
    double      slot_height_mm     = 60.0;           // gate slot height (vertical, perp to flow)
    double      body_y_span_mm     = 50.0;           // body Y-dimension (slot goes full-through)
    double      stem_bore_dia_mm   = 14.0;           // stem hole diameter
    double      stem_bore_depth_mm = 25.0;           // stem hole depth from top
    double      recess_dia_mm      = 14.0;           // bottom recess diameter (gate park)
    double      recess_depth_mm    = 6.0;            // bottom recess depth below slot floor
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gate_valve_seat_compound
}  // namespace koocadcam::skill
