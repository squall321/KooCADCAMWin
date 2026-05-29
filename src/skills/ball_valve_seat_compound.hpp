#pragma once
// @lat: [[engine/skills#ball_valve_seat_compound]]
//
// ball_valve_seat_compound — ball-valve body internal cavity.
//
// Sub-features (REAL geometric cuts):
//   1. Spherical ball cavity (sphere centered on body intersection).
//   2. Through-bore flow cylinder (axial) — connects inlet and outlet.
//   3. Upstream  PTFE seat-ring groove (annular cut, axial).
//   4. Downstream PTFE seat-ring groove (annular cut, axial, mirror of 3).
//   5. Stem bore — vertical cylindrical hole from top of body to ball cavity.
//   6. Body-bonnet flange counterbore — wider seat at the top for the bonnet
//      retaining flange.
//
// DFM (API 608 / ASME B16.34 — Valves Flanged, Threaded, and Welding-End):
//   - PTFE seat groove depth ≥ 0.6 × seat thickness (ISO 17292 §6.3.4 —
//     PTFE seat retention).
//   - Ball clearance 0.05–0.20 mm (API 608 §6.7 torque/seal compromise).
//   - Through-bore dia ≤ ball cavity dia − 2 mm (seat ring space).
//   - Stem bore dia ≥ 5 mm (API 608 §6.5 torque capacity).
//
// Spec table (API 608 Table 4): DN15 / DN20 / DN25 / DN32 / DN40 / DN50.
//
// Recognize: spherical face + flow-axis cylinder + perpendicular stem
// cylinder.  Metadata replay path matches signature exactly.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ball_valve_seat_compound {

constexpr const char* kSkillId = "ball_valve_seat_compound";

struct Input
{
    FaceDatum   entry_face;                          // top face for bonnet flange + stem
    double      center_x_mm        = 0.0;
    double      center_y_mm        = 0.0;
    double      center_z_mm        = 0.0;
    gp_Dir      flow_axis          { 1.0, 0.0, 0.0 };
    gp_Dir      stem_axis          { 0.0, 0.0, 1.0 };

    // Spec key — sizes from API 608 Table 4.
    //   "DN15" | "DN20" | "DN25" | "DN32" | "DN40" | "DN50"
    std::string size_spec          = "DN25";

    // Overrides — when 0.0, the spec-table value is used.
    double      ball_dia_mm        = 0.0;
    double      port_dia_mm        = 0.0;
    double      seat_groove_id_mm  = 0.0;
    double      seat_groove_od_mm  = 0.0;
    double      seat_groove_depth_mm = 0.0;
    double      stem_bore_dia_mm   = 0.0;
    double      stem_bore_depth_mm = 0.0;
    double      flange_recess_dia_mm   = 0.0;
    double      flange_recess_depth_mm = 0.0;

    double      ball_clearance_mm  = 0.10;          // API 608 §6.7
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ball_valve_seat_compound
}  // namespace koocadcam::skill
