#pragma once
// @lat: [[engine/skills#flange]]
//
// flange — a specialized 90° bend at the EDGE of a sheet, forming a
// perpendicular lip.  Often used for fastening tabs, stiffeners, or
// safety hems on panel edges.
//
//                   .─── flange (perpendicular lip)
//                   │   ↑ flange_width_mm
//                   │
//          .────────.────────────────
//                       sheet
//          ─────────────────────────.
//
// The flange edge is one of the four outer edges of the sheet
// (x_min / x_max / y_min / y_max).  The strip nearest that edge of
// width `flange_width_mm` is rotated 90° around its inner edge.
//
// Signature pattern (like bend):
//   - 2 planar faces (main sheet flat + flange flat) at ~90°
//   - 1 cylindrical bend face along the sheet's chosen edge
//   - The flange flat face has area ≈ flange_width × edge_length
//
// DFM checks:
//   DFM-F-MIN-R   : bend_radius_mm ≥ 0.5 × sheet_thickness
//   DFM-F-WIDTH   : flange_width_mm > 0
//   DFM-F-SHEET   : sheet thickness ≤ 5 mm
//
// Recognize:
//   A planar face perpendicular to the main sheet flat, joined via a
//   cylindrical bend, whose area is ≪ the main sheet area.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace flange {

constexpr const char* kSkillId = "flange";

// Which sheet edge to flange.
enum class EdgeSide { XMin, XMax, YMin, YMax };

inline std::string edgeSideToString(EdgeSide s)
{
    switch (s) {
        case EdgeSide::XMin: return "x_min";
        case EdgeSide::XMax: return "x_max";
        case EdgeSide::YMin: return "y_min";
        case EdgeSide::YMax: return "y_max";
    }
    return "x_min";
}

inline EdgeSide edgeSideFromString(const std::string& s)
{
    if (s == "x_max") return EdgeSide::XMax;
    if (s == "y_min") return EdgeSide::YMin;
    if (s == "y_max") return EdgeSide::YMax;
    return EdgeSide::XMin;
}

struct Input
{
    EdgeSide  edge_selector    = EdgeSide::XMax;
    double    flange_width_mm  = 5.0;
    double    bend_radius_mm   = 1.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace flange
}  // namespace koocadcam::skill
