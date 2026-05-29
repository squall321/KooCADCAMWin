#pragma once
// @lat: [[engine/skills#pulley_with_keyway_compound]]
//
// pulley_with_keyway_compound — COMPOUND macro that rebuilds a cylindrical
// stock into a finished pulley:
//
//   sub-cuts (4):
//     1. central bore                    (axial Ø bore_dia)
//     2. circumferential V-groove        (the pulley belt-seat)
//     3. keyway slot inside the bore     (rectangular, run-through axial)
//     4. two balance-drill holes         (Ø 5 × full thickness, offset 180°)
//
// All cuts share the workpiece axis (Z).  Concentric / coaxial tools are
// fused-first-then-cut where required to avoid OCCT's boolean interior-
// overlap edge case.
//
// DFM gates (DIN 6885 keyway / SAE J1313 V-belt):
//   DFM-INPUT, DFM-002 (min wall),
//   DFM-PULLEY-BORE   bore_dia < 6 mm
//   DFM-PULLEY-WALL   (outer_dia - bore_dia)/2 < 5 mm
//   DFM-PULLEY-ANGLE  groove_angle outside [30°, 50°]  (SAE J1313 typical 34°/38°)
//   DFM-KEYWAY-FIT    key_w >= bore_dia / 2            (key bigger than half bore)
//   DFM-KEYWAY-DEPTH  key_h <= 0                       (degenerate)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pulley_with_keyway_compound {

constexpr const char* kSkillId = "pulley_with_keyway_compound";

struct Input
{
    double outer_dia_mm        = 0.0;   // pulley outer Ø (stock OD before V-groove)
    double groove_angle_deg    = 38.0;  // V-belt groove included angle (SAE J1313: 34/38)
    double groove_depth_mm     = 0.0;   // radial depth of V-groove (≤ wall thickness)
    double thickness_mm        = 0.0;   // axial thickness of the pulley puck
    double key_w_mm            = 0.0;   // keyway width        (DIN 6885)
    double key_h_mm            = 0.0;   // keyway extra depth past bore wall
    double bore_dia_mm         = 0.0;   // through-bore Ø
    double balance_drill_dia_mm = 5.0;  // balance hole Ø (defaults to 5 mm)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pulley_with_keyway_compound
}  // namespace koocadcam::skill
