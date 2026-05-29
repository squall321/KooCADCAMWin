#pragma once
// @lat: [[engine/skills#x_ring_groove]]
//
// x_ring_groove — COMPOUND MACRO: X-ring (quad-ring / Quattro) groove on
// the OD of a cylindrical shaft, with chamfered sidewalls and a wider
// rectangular profile than an O-ring (the X-ring's 4-lobe geometry needs
// extra clearance for proper seating).
//
//   ┌─────────────────────────┐  R_outer (stock outer)
//   │                         │
//   ────┐ 10° sidewall ┌────  ← position_z + width/2
//       │╲             ╱│
//       │ ╲           ╱ │      X-ring nominal depth G_x ≈ 0.78·CS
//       │  ╲_________╱  │      width W_x ≈ 1.70·CS  (vs O-ring 1.45)
//   ────┘               └────  ← position_z - width/2
//
// Sub-feature chain (geometric):
//   1. CENTER GROOVE — annular ring of full width W_x and depth G_x.
//   2. SIDE CHAMFER #1 — 10° draft cone-frustum cut at upper sidewall.
//   3. SIDE CHAMFER #2 — 10° draft cone-frustum cut at lower sidewall.
//
// Reference: Parker Quad Seal Handbook (Table 5-1, dynamic radial X-ring):
//   G (depth)  ≈ 0.78 · CS_x   (slightly more squeeze than O-ring)
//   W (width)  ≈ 1.70 · CS_x   (wider — accommodates 4-lobe profile)
//   Sidewall angle ≈ 10° (vs O-ring's ~5°) — eases installation and
//                                            allows for lobe deflection.
//
// X-ring size codes are NOT the same as AS568 — Parker uses Q1 / Q2 / Q3
// series.  We use a "Q-series" code in this skill:
//   "Q1" → CS 1.78 mm    "Q2" → CS 2.62 mm    "Q3" → CS 3.53 mm
//   "Q4" → CS 5.33 mm    "Q5" → CS 6.99 mm
//
// Signature pattern: kind = "x_ring_groove", is_compound = true,
// subfeature_count = 3.
//
// DFM gates:
//   - x_ring_size must be in the lookup table.
//   - mean_dia + G_x must be < (shaft_dia - 1.0 mm wall margin).
//   - mean_dia > 0.

#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace x_ring_groove {

constexpr const char* kSkillId = "x_ring_groove";

struct Input
{
    double      bore_or_shaft_dia_mm = 0.0;     // stock outer diameter
    double      mean_dia_mm          = 0.0;     // groove centerline diameter
    double      position_z_mm        = 0.0;     // axial center of the groove
    std::string x_ring_size;                    // Parker Q-series code, e.g. "Q2"
};

double crossSectionForQ(const std::string& q_code);
double recommendedXDepthFor(double cs_mm);
double recommendedXWidthFor(double cs_mm);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace x_ring_groove
}  // namespace koocadcam::skill
