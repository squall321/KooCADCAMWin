#pragma once
// @lat: [[engine/skills#notch]]
//
// notch — small triangular or rectangular cut at the edge of a sheet.
// Removes material from the sheet's perimeter.  Used as a tab-relief
// before a bend, as a register for assembly, or as a hand-grip relief.
//
//          ┌────────────────────┐
//          │                    │
//          │                    ├──┐  ← rectangular notch (Shape::Rectangle)
//          │                    │  │      width_mm × depth_mm
//          │                    ├──┘
//          │             ╲      │
//          │              ╲     │   ← triangular notch (Shape::Triangle)
//          │              ╱     │      width_mm (mouth) × depth_mm
//          │             ╱      │
//          └────────────────────┘
//
// Geometry strategy:
//   1. From `edge_selector` pick the parametric position along the edge
//      (centred at `position_mm` measured from the lower corner).
//   2. Build the cutter as either a box (rectangle) or a triangular prism
//      (triangle), with depth extending INWARD from the chosen edge and
//      extruded fully through the sheet thickness.
//   3. Subtract the cutter.
//
// Signature pattern:
//   - 2 or 3 newly created vertical planar walls (one per notch side)
//   - 1 missing chunk of the sheet's outer perimeter
//   - sheet_thickness preserved everywhere else
//
// DFM checks:
//   DFM-N-MIN-W     : width_mm ≥ 1.5 × sheet_thickness
//   DFM-N-MIN-D     : depth_mm ≥ sheet_thickness
//   DFM-N-MAX-D     : depth_mm ≤ 50% of perpendicular sheet dimension
//   DFM-N-SHEET     : sheet thickness ≤ 5 mm
//   DFM-INPUT       : width / depth / position positive
//
// Recognize:
//   Find vertical walls (planar, normal in XY plane) NEAR the sheet's
//   outer bbox edge that bound a small inset region not covered by the
//   sheet's primary rectangular footprint.

#include "Datum.hpp"
#include "Skill.hpp"
#include "flange.hpp"     // reuse EdgeSide enum

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace notch {

constexpr const char* kSkillId = "notch";

enum class Shape { Rectangle, Triangle };

inline std::string shapeToString(Shape s)
{
    return (s == Shape::Triangle) ? "triangle" : "rectangle";
}

inline Shape shapeFromString(const std::string& s)
{
    return (s == "triangle") ? Shape::Triangle : Shape::Rectangle;
}

using flange::EdgeSide;
using flange::edgeSideToString;
using flange::edgeSideFromString;

struct Input
{
    EdgeSide  edge_selector = EdgeSide::XMax;
    Shape     shape         = Shape::Rectangle;
    double    position_mm   = 0.0;    // along-edge centre offset (from start of edge)
    double    width_mm      = 0.0;    // tangential width at the mouth
    double    depth_mm      = 0.0;    // inward depth from the edge
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace notch
}  // namespace koocadcam::skill
