#pragma once
// @lat: [[engine/skills#micro_milling]]
//
// micro_milling — sub-millimetre end-mill pocketing (0.05 – 0.5 mm dia).
//
// Topologically identical to a small mill_circular_pocket: 1 cylindrical
// wall + 1 flat planar bottom, but the DFM envelope is far stricter than
// the regular pocket skill:
//
//   - diameter_mm ∈ [0.05, 0.5]  (micro regime — solid-carbide micro mill)
//   - depth/dia    ≤ 5           (stiffness wall; above this the cutter
//                                  deflects beyond tolerance OR snaps)
//   - bottom_corner_r_mm = 0      (sharp corners only — corner-radius end
//                                  mills don't exist in this size range)
//
//                       entry_face (planar)
//                              │
//                  ────────────┼──────────────
//                              │
//                       ┌──────┴──────┐    ↑ depth_mm (≤ dia × 5)
//                       │ flat-btm    │    │
//                       └─────────────┘    ↓
//                              ↑
//                          diameter_mm  (0.05 – 0.5 mm)
//
// Signature pattern:
//   - pattern.kind = "micro_milling"
//   - 1 cylindrical face + 1 small planar (bottom) face + 2 circular edges
//   - dia_mm, depth_mm carried in pattern + params
//
// DFM checks:
//   DFM-MICRO-MILL-DIAMIN  : dia < 0.05 mm → error (below practical limit)
//   DFM-MICRO-MILL-DIAMAX  : dia > 0.5 mm  → error (use mill_circular_pocket)
//   DFM-MICRO-MILL-RATIO   : depth/dia > 5 → error (deflection / snap)
//   DFM-INPUT              : dia ≤ 0 or depth ≤ 0 → error
//
// Recognize:
//   Cylindrical face with small radius + flat planar bottom + dia in the
//   micro-mill envelope.  Confidence 0.85 — diameter envelope cleanly
//   differentiates from mill_circular_pocket and micro_drill (drill_hole
//   recognition has its own aspect-ratio gate).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace micro_milling {

constexpr const char* kSkillId = "micro_milling";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      diameter_mm   = 0.0;            // ∈ [0.05, 0.5] mm
    double      depth_mm      = 0.0;            // depth/dia ≤ 5
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace micro_milling
}  // namespace koocadcam::skill
