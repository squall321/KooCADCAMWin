#pragma once
// @lat: [[engine/skills#beading]]
//
// beading — long thin ridge (additive) OR groove (subtractive) along a
// polyline path on the sheet's top face.  Used to stiffen flat panels
// against denting, and decoratively.
//
//                Additive (mode=Ridge)              Subtractive (mode=Groove)
//                      ╱╲                                   ╲╱
//                ─────╱  ╲─────                      ──────╲  ╱──────
//                       sheet                                ──
//
// Geometry strategy:
//   1. Build a stadium-chain cutter (`pr::offsetPolylineCutter`) along the
//      `waypoints` polyline with width = `width_mm` and depth = `depth_mm`.
//   2. If `mode == Mode::Ridge` — fuse the cutter on TOP of the sheet (it
//      becomes a raised ridge of height `depth_mm`).
//      If `mode == Mode::Groove` — subtract the cutter from the sheet's
//      top, leaving a depression of depth `depth_mm`.
//
// Slice-1 simplification: the bead/groove cross-section is rectangular,
// not the conventional semi-circular form.  Real beading rollers produce
// a rounded cross-section but the signature pattern is identical (a
// raised or sunk strip along the path).
//
// Signature pattern:
//   - 1 raised/sunk path strip along `waypoints`
//   - `path_length_mm`, `width_mm`, `depth_mm`, `mode` (ridge|groove)
//
// DFM checks:
//   DFM-BD-MIN-W    : width_mm ≥ 2 × sheet_thickness
//   DFM-BD-MIN-D    : depth_mm ≥ 0.5 × sheet_thickness
//   DFM-BD-MAX-D    : depth_mm ≤ 2 × sheet_thickness
//   DFM-BD-MIN-LEN  : path length ≥ 3 × sheet_thickness
//   DFM-BD-SHEET    : sheet thickness ≤ 5 mm
//   DFM-INPUT       : waypoints.size() ≥ 2, no duplicate points
//
// Recognize:
//   Find a thin planar +Z face whose centre Z is offset from main sheet
//   top by ~ `depth_mm` (above for ridge, below for groove).  Estimate
//   the path length & width from the face's planar extent.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace beading {

constexpr const char* kSkillId = "beading";

enum class Mode { Ridge, Groove };

inline std::string modeToString(Mode m)
{
    return (m == Mode::Groove) ? "groove" : "ridge";
}

inline Mode modeFromString(const std::string& s)
{
    return (s == "groove") ? Mode::Groove : Mode::Ridge;
}

struct Input
{
    std::vector<std::array<double, 2>>  waypoints;
    double  width_mm = 0.0;
    double  depth_mm = 0.0;
    Mode    mode     = Mode::Ridge;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace beading
}  // namespace koocadcam::skill
