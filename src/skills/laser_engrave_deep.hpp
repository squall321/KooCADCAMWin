#pragma once
// @lat: [[engine/skills#laser_engrave_deep]]
//
// laser_engrave_deep — Deep-engraving laser pocket.
//
// A pulsed fiber/CO2 laser ablates material from a 2D footprint, producing
// a SHALLOW POCKET (depth 0.1–2.0 mm).  Distinct from `laser_mark` (depth
// in microns; cosmetic only) and `laser_cut` (through-cut, no bottom):
// laser_engrave_deep leaves a discernible pocket with a flat-ish bottom.
//
// Geometry — rectangular OR circular floor; depth in the 0.1–2.0 mm band.
//
//                       entry_face (planar)
//                            │
//                ────────────┼──────────────
//                            │     ↓ laser axis
//                  ┌─────────┴─────────┐
//                  │  (rectangular OR  │   ↑ depth 0.1–2 mm
//                  │   circular floor) │   ↓
//                  └───────────────────┘
//
// Signature pattern:
//   shape = "rectangular" → 4 vertical walls + 1 flat bottom
//   shape = "circular"    → 1 cylindrical wall + 1 flat bottom
//
// DFM checks:
//   DFM-INPUT          shape ∉ {"rectangular","circular"}, dim ≤ 0
//   DFM-LE-DEPTH-LO    depth < 0.1 mm → error (use laser_mark)
//   DFM-LE-DEPTH-HI    depth > 2.0 mm → error (use mill_pocket or wire EDM)
//
// Recognize:
//   Metadata-replay only — same disambiguation problem as USM/ECM (no
//   geometric signature that distinguishes laser ablation from milling).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace laser_engrave_deep {

constexpr const char* kSkillId      = "laser_engrave_deep";
constexpr double      kMinDepth_mm  = 0.1;
constexpr double      kMaxDepth_mm  = 2.0;

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    std::string shape;                  // "rectangular" | "circular"
    double      length_mm      = 0.0;   // for "rectangular"
    double      width_mm       = 0.0;   // for "rectangular"
    double      diameter_mm    = 0.0;   // for "circular"
    double      depth_mm       = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace laser_engrave_deep
}  // namespace koocadcam::skill
