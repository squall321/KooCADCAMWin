#pragma once
// @lat: [[engine/skills#blanking]]
//
// blanking — cut a defined-shape blank out of a sheet, keeping ONLY the
// material INSIDE the blank polygon and discarding everything outside.
// This is the inverse of a regular subtractive cut: a blank stamp slices
// out a small shaped piece from a larger sheet so that the *blank* (not the
// skeleton) is the product.
//
// Geometry strategy:
//   - Take the user-supplied closed polygon (XY) → extrude vertically into
//     a tall solid spanning the whole sheet thickness.
//   - INTERSECT the extruded polygon with the sheet (BRepAlgoAPI_Common).
//   - The result is the blank itself.  The skeleton is the scrap and is
//     discarded.
//
// Signature pattern:
//   - shape  = "rectangle" | "circle"
//   - blank_area_mm2 (positive)
//   - 1 closed bounding outline on top + 1 on bottom
//   - is_blanked = true
//
// DFM checks:
//   DFM-BLK-SIZE   : at least 3 points OR positive radius
//   DFM-BLK-SHEET  : stock thickness ≤ 5 mm
//   DFM-BLK-MIN-W  : min blank feature ≥ 1.5 × sheet_thickness
//   DFM-INPUT      : polygon area > 0
//
// Recognize:
//   Workpiece bbox area in XY equals reported blank_area within tolerance
//   AND footprint is tight (top face area matches XY bbox area).

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace blanking {

constexpr const char* kSkillId = "blanking";

enum class Shape { Rectangle, Circle };

inline std::string shapeToString(Shape s)
{
    return (s == Shape::Circle) ? "circle" : "rectangle";
}

struct Input
{
    Shape   shape          = Shape::Rectangle;

    // Rectangle: center + length + width (axis-aligned).
    // Circle:    center + diameter.
    double  center_x_mm    = 0.0;
    double  center_y_mm    = 0.0;
    double  length_mm      = 0.0;     // rectangle X extent
    double  width_mm       = 0.0;     // rectangle Y extent
    double  diameter_mm    = 0.0;     // circle
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace blanking
}  // namespace koocadcam::skill
