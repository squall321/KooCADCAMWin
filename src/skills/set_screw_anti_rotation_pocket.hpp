#pragma once
// @lat: [[engine/skills#set_screw_anti_rotation_pocket]]
//
// set_screw_anti_rotation_pocket — COMPOUND MACRO for a shaft-locking
// set-screw boss commonly seen on collars, hubs, and pulleys.
//
// Sub-feature chain (3 sub-cuts):
//   1. Tapped set-screw hole         — vertical pilot pilot_dia (table) at
//                                      (position).  Depth = boss thickness
//                                      derived from shaft_intersect_dia
//                                      and the workpiece bbox.
//   2. Perpendicular shaft pilot     — horizontal cylinder along the
//      cross-drill                    perpendicular axis at the same XY,
//                                      diameter = shaft_intersect_dia.
//                                      This is where the locking set-screw
//                                      bites into the shaft.
//   3. Chamfer at set-screw mouth    — short cone frustum (1×45° per
//                                      DIN 76 entry chamfer) at the top
//                                      to guide the screw and prevent
//                                      thread-stripping at the rim.
//
//                          set-screw axis (axis_dir, typically -Z)
//                                  │
//                                  ▼ ─── chamfer (cone frustum, sub-feature 3)
//                      ────┬──────────────┬──── entry_face
//                          │ pilot dia    │
//                          │              │       ◄── set-screw hole (sub 1)
//                          │              │
//                       ───┤              ├───
//                          │              │    ── shaft cross-drill (sub 2)
//                       ───┤              ├───
//                          │              │
//                          └──────────────┘
//
// Standard:
//   - ISO 4029 (cup-point set-screw) — entry chamfer recommended.
//   - SAE J429 (US headless) — equivalent practice.
//
// DFM:
//   - Set-screw size in {M3, M4, M5, M6, M8}.
//   - shaft_intersect_dia > set_screw_pilot_dia (must clear bore for screw
//     to bite shaft).
//   - shaft_intersect_dia ≤ workpiece bbox minor dim minus 2 mm wall.
//   - Chamfer dia > pilot dia.
//
// Signature: is_compound = true, subfeature_count = 3.
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace set_screw_anti_rotation_pocket {

constexpr const char* kSkillId = "set_screw_anti_rotation_pocket";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm        = 0.0;
    double      position_y_mm        = 0.0;
    gp_Dir      axis_dir             { 0.0, 0.0, -1.0 };   // set-screw axis
    gp_Dir      shaft_axis_dir       { 1.0, 0.0, 0.0  };   // shaft (cross-drill) axis
    std::string set_screw_M;                                // "M3", "M4", "M5", "M6", "M8"
    double      shaft_intersect_dia  = 0.0;                 // cross-drill diameter
    double      chamfer_size_mm      = 0.5;                 // 0.5 mm × 45°
    double      boss_thickness_mm    = 0.0;                 // depth of set-screw hole
};

double setScrewPilotDiameterFor(const std::string& set_screw_M);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace set_screw_anti_rotation_pocket
}  // namespace koocadcam::skill
