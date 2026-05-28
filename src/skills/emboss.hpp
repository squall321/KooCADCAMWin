#pragma once
// @lat: [[engine/skills#emboss]]
//
// emboss — localized vertical deformation (raised feature) on a sheet's
// top face.  Used for logos, stiffening ribs, tactile markers.
//
//                  ┌────────┐                ↑ height_mm
//                  │        │  (cylinder OR  │
//                ──┴────────┴──    rounded   │
//                       sheet      box)
//                                                ↑ sheet_thickness
//                ─────────────────────────────
//
// Slice-1 simplification: the emboss is a purely-additive raised feature
// on the top face.  Real embossing also deforms the bottom face (it's a
// localized depression mirror), but here we keep the bottom flat.
//
// Signature pattern:
//   - 1 cylindrical or rounded-rectangular face protruding above the top
//   - 1 top capping face (planar, parallel to sheet)
//   - shape = "circle" | "rectangle"
//
// DFM checks:
//   DFM-E-DEPTH   : height_mm ≤ 3 × sheet_thickness (deep-draw limit)
//   DFM-E-CORNER  : corner_r_mm ≥ sheet_thickness (for rect)
//   DFM-E-SHEET   : sheet thickness ≤ 5 mm
//
// Recognize:
//   Cylindrical OR rounded-rect-corner face protruding from the largest
//   planar (+Z) face, axis perpendicular.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace emboss {

constexpr const char* kSkillId = "emboss";

enum class Shape { Circle, Rectangle };

inline std::string shapeToString(Shape s)
{
    return (s == Shape::Circle) ? "circle" : "rectangle";
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

    // For circle: diameter_mm is used.  For rectangle: length_mm & width_mm.
    double  diameter_mm   = 0.0;
    double  length_mm     = 0.0;
    double  width_mm      = 0.0;

    double  height_mm     = 0.0;     // raised height above sheet top
    double  corner_r_mm   = 0.0;     // for rectangle only
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace emboss
}  // namespace koocadcam::skill
