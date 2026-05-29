#pragma once
// @lat: [[engine/skills#ball_screw_nut_pocket]]
//
// ball_screw_nut_pocket — COMPOUND MACRO for a ball-screw nut housing.
// Combines:
//   1. A cylindrical pocket sized for slip-fit on the nut OD (slip clearance).
//   2. Four mounting holes around the OD (configurable pattern).
//   3. A center through-pocket (smaller dia) for the lead screw shaft.
//
//              ┌─────────────────────────┐
//              │     ○             ○     │   ← 4 mounting holes
//              │      ╭─────────────╮    │
//              │      │  nut bore    │   │
//              │      │  (slip OD)  │    │
//              │      ╰─────────────╯    │
//              │     ○             ○     │
//              └─────────────────────────┘
//                       │ through-bore (screw OD) │
//
// Sub-feature count = 1 nut pocket + 4 mounting holes + 1 thru-pocket = 6.
//
// Standards reference: THK/HIWIN/NSK ball-screw nut flange specs (4-bolt PCD).
// Slip-fit clearance: H7/h6 ~ 25 μm; we use 0.05 mm in geometry.
//
// DFM checks:
//   DFM-INPUT      : nut_od, nut_l > 0
//   DFM-002        : derived shaft thru-dia >= 0.8 mm
//   DFM-BS-NUTOD   : nut_od in [10, 80] mm (warning otherwise)
//   DFM-BS-PCD     : mounting PCD must fit OD - mounting_hole_dia
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ball_screw_nut_pocket {

constexpr const char* kSkillId = "ball_screw_nut_pocket";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm       = 0.0;   // pocket center
    double      position_y_mm       = 0.0;
    gp_Dir      axis_dir            { 0.0, 0.0, -1.0 };
    double      nut_od_mm           = 0.0;   // nut outer diameter
    double      nut_l_mm            = 0.0;   // nut length (pocket depth)
    // "square4" → 4 holes at 90° on PCD; "diamond4" → rotated 45°
    std::string mounting_pattern    = "square4";
    double      mounting_pcd_mm     = 0.0;   // 0 → derived (= nut_od + 8 mm)
    double      mounting_hole_dia_mm = 5.5;  // for M5 mounting
    // Shaft thru-dia defaults to nut_od / 2 - 1.0 (typical for the screw root)
    double      shaft_thru_dia_mm   = 0.0;   // 0 → derived
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ball_screw_nut_pocket
}  // namespace koocadcam::skill
