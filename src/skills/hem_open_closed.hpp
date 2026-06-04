#pragma once
// @lat: [[engine/skills#hem_open_closed]]
//
// hem_open_closed — Production-grade sheet-metal hem with selectable style:
//
//   closed  : gap = 0           (flat against parent material)
//   open    : gap = R / 2       (small air gap for paint flow)
//   teardrop: gap = R           (large drop-shaped pocket)
//
// Unlike the basic `hem` skill (which only handles flat/open at axis-aligned
// edges), this version accepts an arbitrary polyline edge in XY and folds
// the strip back along it with style-driven gap.
//
// Synthesis:
//   1. Compute the unit tangent along each polyline segment + the inward
//      normal in XY.
//   2. For each segment add a hem strip:
//        - a 180° cylindrical-shell roll of inner radius hem_radius_mm
//          (the wrap)
//        - a flat back-slab of length `hem_length_mm - π·R`
//          offset by (sheet_thickness + gap) above the parent top face.
//   3. Fuse all strips into the workpiece.
//
// DFM:
//   DFM-HOC-SHEET   : 0.5 ≤ thickness ≤ 6.0 mm
//   DFM-HOC-RADIUS  : hem_radius_mm ≥ thickness
//   DFM-HOC-LENGTH  : hem_length_mm ≥ 2 × thickness + π × hem_radius_mm
//   DFM-HOC-POLY    : edge_polyline has ≥ 2 points
//
// Recognize:
//   Local parallel planar pair offset above sheet top, with one cylindrical
//   180° wrap face whose axis is in the sheet plane.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <gp_Pnt.hxx>

namespace koocadcam::skill {

class Workpiece;

namespace hem_open_closed {

constexpr const char* kSkillId = "hem_open_closed";

enum class HemStyle
{
    Closed,    // gap = 0
    Open,      // gap = radius / 2
    Teardrop,  // gap = radius
};

inline const char* hemStyleToString(HemStyle s)
{
    switch (s) {
        case HemStyle::Closed:   return "closed";
        case HemStyle::Open:     return "open";
        case HemStyle::Teardrop: return "teardrop";
    }
    return "closed";
}

inline HemStyle hemStyleFromString(const std::string& s)
{
    if (s == "open")     return HemStyle::Open;
    if (s == "teardrop") return HemStyle::Teardrop;
    return HemStyle::Closed;
}

struct Input
{
    std::vector<gp_Pnt> edge_polyline;          // XY polyline in world coords
    HemStyle            hem_style    = HemStyle::Closed;
    double              hem_radius_mm = 1.0;
    double              hem_length_mm = 6.0;    // total wrapped length
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hem_open_closed
}  // namespace koocadcam::skill
