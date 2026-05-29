#pragma once
// @lat: [[engine/skills#o_ring_groove_radial]]
//
// o_ring_groove_radial — COMPOUND MACRO: cylindrical-face O-ring groove
// per AS568 catalog (rectangular cross-section + bottom radius relief +
// side-wall draft).
//
//   ┌─────────────────────────┐  R_outer (stock outer)
//   │                         │
//   ────┐  draft 5°    ┌────  ← z = position_z + width/2
//       │   ╲       ╱  │
//       │   ╲      ╱   │      groove depth = G  (from outer surface)
//       │    ╲____╱    │      bottom radius = b_r (≈ 5% of W)
//   ────┘              └────  ← z = position_z - width/2
//                ↑
//             position_z
//
// Sub-feature chain (geometric):
//   1. MAIN GROOVE — annular ring (rect cross-section), full width W and
//      depth G, removed via pr::annularRing + pr::cut.
//   2. BOTTOM RADIUS — small annular relief at groove bottom, providing the
//      5%-W fillet seat for the O-ring (AS568 recommends b_r ≈ 0.05·W).  Cut
//      as an annular ring of width = 2·b_r and depth = b_r, fused with the
//      main groove then subtracted in one boolean.
//   3. SIDE DRAFT — two cone-frustum lead-in chamfers at top + bottom of the
//      groove (5° draft from groove ID outward, helps O-ring installation).
//
// Reference: AS568 / SAE AS568A standard catalog.  Cross-reference dash
// numbers map to O-ring CS (cross-section):
//   "-006" → CS 1.78 mm    "-014" → CS 1.78 mm
//   "-110" → CS 2.62 mm    "-210" → CS 3.53 mm    "-310" → CS 5.33 mm
//   "-220" → CS 3.53 mm    "-225" → CS 3.53 mm    "-228" → CS 3.53 mm
//
// Per Parker O-ring Handbook (table 4-1, radial static seal):
//   groove depth G ≈ 0.74·CS    (78%·CS for static dynamic)
//   groove width W ≈ 1.45·CS    (squeeze + extrusion gap)
//
// Signature pattern: kind = "o_ring_groove_radial", is_compound = true,
// subfeature_count = 3.
//
// DFM gates (AS568):
//   - o_ring_size must be in the lookup table.
//   - groove depth G > 0 and < (bore_radius - 1.0 mm) — keep 1 mm wall.
//   - bore_or_shaft >= 2.0 mm to accommodate groove + 1 mm wall.

#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace o_ring_groove_radial {

constexpr const char* kSkillId = "o_ring_groove_radial";

struct Input
{
    // Outer-surface diameter of the shaft (or bore, if external groove on bore wall).
    double      bore_or_shaft_dia_mm = 0.0;
    double      position_z_mm        = 0.0;   // axial center of the groove
    std::string o_ring_size;                  // AS568 dash number, e.g. "-110"
};

// AS568 → (CS, OD) lookup.  Returns CS mm; 0 if unknown.
double crossSectionFor(const std::string& as568_dash);

// Recommended groove depth & width from CS (Parker Handbook, radial static).
double recommendedDepthFor(double cs_mm);
double recommendedWidthFor(double cs_mm);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace o_ring_groove_radial
}  // namespace koocadcam::skill
