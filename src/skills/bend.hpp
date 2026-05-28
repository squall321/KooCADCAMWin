#pragma once
// @lat: [[engine/skills#bend]]
//
// bend — fold a sheet along a line by a given angle.
//
//                                      ╱
//                                     ╱  ← rotated half (angle_deg)
//                                    ╱
//                                  bend zone
//                                   ─ (cylindrical, inside radius)
//                  ──────────────────
//                       fixed half
//
// The bend axis is the bend_line itself, lying in the top-plane of the
// sheet (Z = z_top).  Positive `angle_deg` rotates the FAR half upward
// (its outward normal swings toward +Z then beyond).
//
// Geometry strategy (slice-1 approximation; produces a valid solid):
//   1. Identify the bend_line in world XY at the top face of the sheet.
//   2. Cut the sheet by a thin planar cutter at the bend_line into two
//      halves: H_fixed (on the LEFT of the bend_line direction) and
//      H_rotated (on the RIGHT).
//   3. Apply gp_Trsf::SetRotation(bend_axis, angle_rad) to H_rotated.
//   4. Build a cylindrical-sector "bend wedge" along the bend axis with
//      inside radius = bend_radius_mm, outside radius = bend_radius_mm +
//      sheet_thickness, spanning angle_deg.
//   5. Fuse H_fixed + bend_wedge + H_rotated into one solid.
//
// Signature pattern:
//   - 2 large planar faces (the flats), one on each side of the bend
//   - 1 cylindrical bend face whose axis = bend axis, radius = bend_radius
//   - The angle between the two flat normals = (180° − |angle_deg|) for
//     interior-angle conventions, or = |angle_deg| if measured between
//     unfolded references.
//
// DFM checks:
//   DFM-B-MIN-R     : bend_radius_mm ≥ 0.5 × sheet_thickness
//   DFM-B-ANGLE     : angle_deg ∈ [-180, 180]
//   DFM-INPUT       : bend_line endpoints distinct
//
// Recognize:
//   Find a cylindrical face that is NOT a hole (its axis lies on the
//   sheet, not perpendicular to it) and which is bounded by TWO planar
//   faces (the flats).  The bend axis is the cylinder axis.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bend {

constexpr const char* kSkillId = "bend";

struct Input
{
    std::array<double, 2>  bend_line_start_xy { 0.0, 0.0 };
    std::array<double, 2>  bend_line_end_xy   { 0.0, 0.0 };
    double                 angle_deg     = 90.0;  // positive = upward fold
    double                 bend_radius_mm = 1.0;  // inside radius
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Helper: sheet thickness from bbox (the smallest extent).
double sheetThickness(const Workpiece& wp);

}  // namespace bend
}  // namespace koocadcam::skill
