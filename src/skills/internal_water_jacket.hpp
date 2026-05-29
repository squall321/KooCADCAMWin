#pragma once
// @lat: [[engine/skills#internal_water_jacket]]
//
// internal_water_jacket — COMPOUND MACRO that machines a cooling water
// jacket on the inside of a bore: a large helical groove around the bore
// wall, plus an inlet port and outlet port (small radial drilled holes
// that connect from the outer surface to the jacket).
//
//                          ┌──────── outer wall ────────┐
//                          │                            │
//                          │    .─────.   .─────.       │
//                          │   (  in   ) (  out  )      │
//                          │    `─────'   `─────'       │
//                          │      │           │         │
//                          │      ▼           ▼         │
//                          │   ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲         │
//                          │   ╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱         │
//                          │   ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲  ← jacket
//                          │   ╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱      groove
//                          │   ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲         │
//                          │                            │
//                          └────────────────────────────┘
//                                       ▲
//                                       │ bore_axis
//
// Sub-feature chain (3 primitive cuts):
//   1. Helical sweep groove on inner bore wall (pitch × groove_w jacket).
//   2. Inlet port — radial drilled hole from outer surface to the jacket.
//   3. Outlet port — radial drilled hole, axially offset from the inlet.
//
// DFM gates:
//   DFM-INPUT             : non-positive sizes.
//   DFM-002               : port_dia_mm < 0.8.
//   DFM-JACKET-PITCH      : pitch < 2 × groove_w (turns would overlap).
//   DFM-JACKET-GEOM       : groove_w >= outer_thickness (wall would breach).
//   DFM-JACKET-PORT-CLEAR : inlet_z == outlet_z within groove_w (ports
//                           clash within one turn).
//
// Recognize: metadata replay (helical sub-surface fingerprint is impossible
// to discriminate from any thread without context).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace internal_water_jacket {

constexpr const char* kSkillId = "internal_water_jacket";

struct Input
{
    FaceDatum   reference_face;                              // outer face for datum reference
    gp_Dir      bore_axis           { 0.0, 0.0, 1.0 };       // local Z of bore
    double      bore_origin_x_mm    = 0.0;                   // bore axis XY in world
    double      bore_origin_y_mm    = 0.0;
    double      bore_axial_start_mm = 0.0;                   // along bore_axis
    double      bore_inner_r_mm     = 10.0;                  // inside radius (the existing bore)
    double      outer_r_mm          = 20.0;                  // body outer radius
    double      jacket_length_mm    = 30.0;                  // axial length of jacket region
    double      jacket_pitch_mm     = 6.0;                   // helical pitch
    double      groove_w_mm         = 3.0;                   // groove width (axial)
    double      groove_depth_mm     = 2.0;                   // radial depth
    double      port_dia_mm         = 3.0;                   // inlet/outlet radial hole dia
    double      inlet_axial_pos_mm  = 2.0;                   // axial pos from bore_axial_start
    double      outlet_axial_pos_mm = 28.0;                  // axial pos from bore_axial_start
    double      inlet_clock_deg     = 0.0;                   // angular position around axis
    double      outlet_clock_deg    = 180.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace internal_water_jacket
}  // namespace koocadcam::skill
