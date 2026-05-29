#pragma once
// @lat: [[engine/skills#banjo_fitting_seat]]
//
// banjo_fitting_seat — COMPOUND MACRO for a banjo-style hydraulic fitting
// (DIN 7642 / SAE J1467 / motorcycle brake-line geometry): a through-bolt
// hole that secures the banjo body, sealing-washer counterbores on BOTH
// sides of the hole (for the copper crush washers), and a cross-drilled
// fluid passage that meets the bolt hole inside the banjo body to channel
// fluid between the bolt's transverse drilling and the host port.
//
//                    bolt_M (vertical)
//                          │
//          ───┬─────────────────────────┬───
//             │  ╔═══washer cbore═══╗   │      ← +Z face washer cbore (sub-cut #2)
//             │  ║                  ║   │
//             │  ║       ┌─┐        ║   │
//             │  ╚═══════╪═╪════════╝   │
//             │          │ │            │
//             │          │ │            │      ← bolt through-hole (sub-cut #1)
//             │   ━━━━━━━╪═╪━━━━━━━     │      ← cross fluid passage (sub-cut #4)
//             │          │ │            │
//             │  ╔═══════╪═╪════════╗   │
//             │  ║       └─┘        ║   │
//             │  ║                  ║   │
//             │  ╚═══════════════════╝  │      ← -Z face washer cbore (sub-cut #3)
//          ───┴─────────────────────────┴───
//
// Sub-feature chain (4 primitive cuts):
//   1. Through-hole at bolt_M pilot diameter.
//   2. Top-face counterbore for the upper crush washer.
//   3. Bottom-face counterbore for the lower crush washer.
//   4. Cross-drilled fluid passage at banjo_dia, perpendicular to bolt axis.
//
// DFM gates:
//   DFM-INPUT       : non-positive sizes.
//   DFM-002         : bolt pilot / banjo_dia < 0.8 mm.
//   DFM-BANJO-SEAT  : washer_seat_dia <= bolt_dia + 2 × wall (no metal under washer).
//   DFM-BANJO-FLOW  : banjo_dia >= bolt_pilot_dia (cross-drill would sever bolt).
//   DFM-BANJO-GEOM  : two cbores overlap (part too thin).
//
// Recognize: metadata replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace banjo_fitting_seat {

constexpr const char* kSkillId = "banjo_fitting_seat";

struct Input
{
    FaceDatum   face_id;
    double      position_x_mm     = 0.0;
    double      position_y_mm     = 0.0;
    gp_Dir      bolt_axis         { 0.0, 0.0, -1.0 };
    std::string bolt_M;                                    // "M6" .. "M14"
    double      banjo_dia_mm      = 4.0;                   // cross-drilled passage dia
    double      banjo_axial_z_mm  = 0.0;                   // axial pos of cross-drill along bolt_axis
    double      washer_seat_depth_mm = 1.5;                // each cbore depth
    double      part_thickness_mm = 12.0;                  // for DFM
};

// Resolve bolt_M → (pilot/clearance_dia_mm, washer_seat_dia_mm).
bool resolveBoltM(const std::string& size,
                  double& clearance_dia_mm,
                  double& washer_seat_dia_mm);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace banjo_fitting_seat
}  // namespace koocadcam::skill
