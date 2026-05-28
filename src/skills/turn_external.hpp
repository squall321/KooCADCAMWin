#pragma once
// @lat: [[engine/skills#turn_external]]
//
// turn_external — outer-diameter (OD) turning operation on cylindrical stock.
//
// Reduce the stock's outer radius from R_initial (bbox.outerRadiusXY()) to
// R_final (= final_dia_mm / 2) over the Z range [start_z_mm, end_z_mm].
//
//                ┌─────────────────┐  R_initial
//                │                 │
//        ────────┘   ┌─────────┐   └──────── ← end_z (shoulder, normal +Z)
//                    │         │             R_final
//                    │         │
//        ────────────┘         └──────────── ← start_z (shoulder, normal -Z)
//                              │
//
// The synthesis cuts away an annular ring of inner radius = R_final and outer
// radius = current outer radius over the Z range.  The result has:
//
//   - 1 NEW cylindrical face at radius R_final between start_z and end_z
//   - 2 NEW annular planar shoulder faces at start_z (normal -Z) and end_z
//     (normal +Z), each connecting R_final to R_initial.
//
// Signature pattern (what the skill leaves on the workpiece):
//   - 1 cylindrical face (axis = +Z, radius = R_final)
//   - 2 annular planar faces perpendicular to Z, one at start_z, one at end_z
//   - 2 pairs of circular edges (top/bottom of each shoulder ring)
//
// DFM checks:
//   DFM-INPUT  : final_dia_mm > 0, end_z > start_z, both diameters ≥ 0.8 mm
//   DFM-LATHE  : final_dia_mm < 2 × bbox.outerRadiusXY() (must remove material)
//   DFM-SURFSP : surface speed (≈ π × dia × rpm-rule-of-thumb) sanity check
//
// Recognize:
//   Iterate cylindrical faces with axis parallel to Z and radius strictly less
//   than the workpiece bbox outer radius.  For each candidate, verify it is
//   bounded by 2 planar circular edges (perpendicular to Z), and that the
//   workpiece has annular planar faces (normal ±Z) sharing the circles.

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace turn_external {

constexpr const char* kSkillId = "turn_external";

struct Input
{
    double  start_z_mm     = 0.0;   // lower Z bound of turn zone
    double  end_z_mm       = 0.0;   // upper Z bound of turn zone (> start_z_mm)
    double  final_dia_mm   = 0.0;   // outer diameter after turning
    int     roughing_passes = 1;    // informational (tooling meta)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace turn_external
}  // namespace koocadcam::skill
