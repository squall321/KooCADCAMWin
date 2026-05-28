#pragma once
// @lat: [[engine/skills#mold_boss]]
//
// mold_boss — cylindrical post/standoff added to a planar face, optionally
// hollowed for self-tapping screws.
//
//                          outer_dia_mm
//                        ◄──────────────►
//                        ┌──────────────┐  ▲
//                        │░░░░░░░░░░░░░░│  │
//                        │░░░  ┌──┐  ░░░│  │ height_mm
//                        │░░░  │  │  ░░░│  │
//                        │░░░  │  │  ░░░│  │
//                        │░░░  └──┘  ░░░│  │
//                        │  inner_dia_mm│  ↓
//      ──────────────────┴──────────────┴──────────────  entry_face
//
// If inner_dia_mm == 0 → solid boss.  Otherwise the boss is hollow
// (concentric counterbore) — typical for self-tapping screw posts.
//
// Signature pattern:
//   - 1 outer cylindrical face (axis perpendicular to entry_face)
//   - 1 top planar face (annular if hollow, full circle if solid)
//   - (hollow only) 1 inner cylindrical face + 1 inner bottom planar face
//
// DFM:
//   DFM-BOSS-DIA      : outer_dia ≥ 1.5 mm (otherwise too thin to mold).
//   DFM-BOSS-WALL     : if hollow, (outer - inner) ≥ 1.2 mm (wall ≥ 0.6 each side).
//   DFM-BOSS-DRAFT    : draft_angle ≥ 0.5 deg.
//   DFM-BOSS-HEIGHT   : height/outer_dia ratio ≤ 4 (else unstable).

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mold_boss {

constexpr const char* kSkillId = "mold_boss";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    double      height_mm       = 0.0;
    double      outer_dia_mm    = 0.0;
    double      inner_dia_mm    = 0.0;   // 0 = solid; > 0 = hollow boss
    double      draft_angle_deg = 1.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mold_boss
}  // namespace koocadcam::skill
