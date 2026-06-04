#pragma once
// @lat: [[engine/skills#gusset_form]]
//
// gusset_form — Stamped reinforcement gusset (formed rib) in the inside corner
// of two adjacent sheet-metal walls.  Drastically stiffens an L- or U-shape
// without adding a separate part.
//
//             wall_a (vertical)
//             │  ╲      ←  formed dome (on outside corner)
//             │   ╲
//             │    ╲
//             │_____╲___________  wall_b (horizontal)
//                ↑
//              gusset_width
//
// Synthesis:
//   1. Cut a triangular slit on the INSIDE corner (so the sheet can deform).
//   2. Fuse a dome-like raised cap on the OUTSIDE corner — the formed gusset.
//
// DFM:
//   DFM-G-SHEET    : 0.5 ≤ thickness ≤ 6.0 mm
//   DFM-G-WIDTH    : gusset_width_mm  ≥ 3 × thickness
//   DFM-G-DEPTH    : gusset_depth_mm  ≥ 2 × thickness AND ≤ 8 × thickness
//   DFM-G-RADIUS   : gusset_radius_mm ≥ thickness
//
// Recognize:
//   A localized RAISED planar region near a sheet inside-corner adjacent to
//   two orthogonal large planar walls.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gusset_form {

constexpr const char* kSkillId = "gusset_form";

struct Input
{
    std::array<double, 3> corner_xyz       { 0.0, 0.0, 0.0 };
    std::array<double, 3> wall_a_normal    { 1.0, 0.0, 0.0 };  // outward normal
    std::array<double, 3> wall_b_normal    { 0.0, 0.0, 1.0 };
    double                gusset_width_mm  = 6.0;
    double                gusset_depth_mm  = 4.0;
    double                gusset_radius_mm = 1.5;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gusset_form
}  // namespace koocadcam::skill
