#pragma once
// @lat: [[engine/skills#butterfly_valve_disc_seat]]
//
// butterfly_valve_disc_seat — concentric butterfly valve body machining.
//
// Sub-features (REAL cuts):
//   1. Through cylindrical bore (the flow passage).
//   2. Annular O-ring seat groove inside the bore (mid-length, for the
//      resilient seat seal).
//   3. Top stub bearing bore — vertical (perp to flow), accommodates upper
//      stem.
//   4. Bottom stub bearing bore — symmetric to top stub bearing.
//   5. Top stem port — extends top stub bearing through to the outer body
//      surface (driver coupling access).
//
// DFM (API 609 — Butterfly Valves):
//   - Bearing stub OD ≥ flow bore dia × 0.15 (API 609 §6 disc-stem ratio).
//   - O-ring groove must clear bearing-stub axes by ≥ 5 mm (avoid intersect).
//   - Top stem port dia ≤ bearing stub dia (driver coupling fits in bearing).
//
// Recognize: 1 large cylindrical through-face + axial annular groove +
// 2 perpendicular small cylindrical bearing bores.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace butterfly_valve_disc_seat {

constexpr const char* kSkillId = "butterfly_valve_disc_seat";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm           = 0.0;        // body center
    double      center_y_mm           = 0.0;
    double      center_z_mm           = 0.0;
    gp_Dir      flow_axis             { 1.0, 0.0, 0.0 };
    gp_Dir      stem_axis             { 0.0, 0.0, 1.0 };

    double      flow_bore_dia_mm      = 40.0;
    double      flow_bore_len_mm      = 50.0;
    double      seat_groove_id_mm     = 40.0;       // matches bore ID
    double      seat_groove_radial_depth_mm = 2.5;  // depth INTO the wall
    double      seat_groove_width_mm  = 4.0;        // axial width of groove
    double      stub_bearing_dia_mm   = 12.0;       // each stub bearing
    double      stub_bearing_depth_mm = 8.0;        // each bearing depth from bore wall
    double      stem_port_dia_mm      = 10.0;       // top stem port (through-body)
    double      body_y_span_mm        = 60.0;       // overall body Y dimension
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace butterfly_valve_disc_seat
}  // namespace koocadcam::skill
