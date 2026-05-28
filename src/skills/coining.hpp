#pragma once
// @lat: [[engine/skills#coining]]
//
// coining — localised stamping that creates a SHALLOW impression at a
// named XY location.  Used for serial numbers, logos, and DFM relief
// notches.  Unlike `emboss` (which can be quite tall), a coined feature
// is intentionally shallow: depth ≤ 30% of sheet thickness.
//
//                  ┌─────┐
//                ──┘     └──  ← coined depression  ↓ depth_mm
//                                                  (≤ 0.3 × thickness)
//             ───────────────  sheet
//             ───────────────
//
// Geometry strategy:
//   1. Build either a cylindrical (`Shape::Circle`) or rounded-rectangle
//      (`Shape::Rectangle`) cutter at (position_x_mm, position_y_mm) just
//      below the sheet top, of axial extent `depth_mm`.
//   2. Cut the sheet by that cutter — produces a shallow indentation.
//
// Signature pattern:
//   - 1 newly created planar bottom face at (sheet_top − depth_mm)
//   - 1 cylindrical OR rounded-rectangular vertical wall above it
//   - All within sheet_thickness × 30%
//
// DFM checks:
//   DFM-C-MAX-DEPTH : depth_mm ≤ 0.3 × sheet_thickness
//   DFM-C-MIN-DEPTH : depth_mm ≥ 0.05 × sheet_thickness
//   DFM-C-SHEET     : sheet thickness ≤ 5 mm
//   DFM-INPUT       : positive dims & depth
//
// Recognize:
//   Cylindrical (or rect) face protruding DOWN from the sheet's top
//   planar face, of axial extent ≤ 0.3 × thickness, with a small planar
//   bottom face.  Confidence ~ 0.70.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace coining {

constexpr const char* kSkillId = "coining";

enum class Shape { Circle, Rectangle };

inline std::string shapeToString(Shape s)
{
    return (s == Shape::Rectangle) ? "rectangle" : "circle";
}

inline Shape shapeFromString(const std::string& s)
{
    return (s == "rectangle") ? Shape::Rectangle : Shape::Circle;
}

struct Input
{
    double  position_x_mm = 0.0;
    double  position_y_mm = 0.0;
    Shape   shape         = Shape::Circle;

    // For Circle: diameter_mm.  For Rectangle: length_mm, width_mm, corner_r_mm.
    double  diameter_mm   = 0.0;
    double  length_mm     = 0.0;
    double  width_mm      = 0.0;
    double  corner_r_mm   = 0.0;

    double  depth_mm      = 0.0;     // < 0.3 × sheet_thickness
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace coining
}  // namespace koocadcam::skill
