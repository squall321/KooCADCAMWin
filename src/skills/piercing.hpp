#pragma once
// @lat: [[engine/skills#piercing]]
//
// piercing — punch a hole (circular OR rectangular) through a sheet, with
// explicit punch-and-die clearance recorded.
//
// Distinct from `sheet_punch` in two ways:
//   1. piercing supports BOTH circular and rectangular hole shapes,
//   2. piercing records the punch-die clearance in tooling metadata so
//      the process planner can size dies correctly (typical clearance is
//      6–10% of sheet thickness per side).
//
// Signature pattern:
//   - shape = "circle" | "rectangle"
//   - is_through_cut = true
//   - punch_die_clearance_mm (per side)
//
// DFM checks:
//   DFM-PRC-MIN-DIM    : min hole feature ≥ sheet_thickness
//   DFM-PRC-SHEET      : sheet_thickness ≤ 5 mm
//   DFM-PRC-CLEARANCE  : clearance ∈ [3%, 15%] × sheet_thickness
//   DFM-INPUT          : dim > 0
//
// Recognize:
//   Through-cut hole (cylindrical or 4 vertical planar walls) spanning the
//   sheet thickness.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace piercing {

constexpr const char* kSkillId = "piercing";

enum class Shape { Circle, Rectangle };

inline std::string shapeToString(Shape s)
{
    return (s == Shape::Circle) ? "circle" : "rectangle";
}

struct Input
{
    FaceDatum   entry_face;
    Shape       shape          = Shape::Circle;
    double      position_x_mm  = 0.0;
    double      position_y_mm  = 0.0;

    double      diameter_mm    = 0.0;     // circle
    double      length_mm      = 0.0;     // rectangle X
    double      width_mm       = 0.0;     // rectangle Y

    // Clearance per side (mm).  If 0, default to 8% × thickness.
    double      clearance_mm   = 0.0;

    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace piercing
}  // namespace koocadcam::skill
