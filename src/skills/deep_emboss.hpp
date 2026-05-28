#pragma once
// @lat: [[engine/skills#deep_emboss]]
//
// deep_emboss — a deep raised protrusion in sheet metal (1–5 mm vs the
// shallow emboss skill's ≤ 0.5 mm).  Used for stiffening ribs, mounting
// bosses, lock buttons, and other structural raised features that need
// enough draw depth to provide functional rigidity.
//
//                          ┌────────┐                ↑ emboss_depth_mm
//                          │        │                │ (1–5 mm)
//                        ──┴────────┴──              │
//                                   sheet           ↓
//                          ─────────────────────────
//
// Slice-1 simplification: purely additive bump (same as `emboss`); the
// real-process bottom would dimple in.  We DO check the depth/thickness
// ratio for DFM (deep-draw critical limit).
//
// Signature pattern:
//   - kind = "deep_emboss"
//   - emboss_depth_mm
//   - depth_to_thickness_ratio
//
// DFM checks:
//   DFM-DEMB-DEPTH-MIN   : depth ≥ 1.0 mm (otherwise use `emboss`)
//   DFM-DEMB-DEPTH-MAX   : depth ≤ 5 × sheet_thickness (deep-draw critical limit)
//   DFM-DEMB-RATIO       : depth / thickness ≤ 4.0 (tearing warning above)
//   DFM-DEMB-SHEET       : sheet_thickness ≤ 5 mm
//   DFM-INPUT            : diameter > 0
//
// Recognize:
//   Cylindrical face with vertical axis protruding above the sheet top by
//   ≥ 1 mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace deep_emboss {

constexpr const char* kSkillId = "deep_emboss";

struct Input
{
    double  position_x_mm  = 0.0;
    double  position_y_mm  = 0.0;
    double  diameter_mm    = 0.0;
    double  emboss_depth_mm = 0.0;     // protrusion height (1–5 mm typical)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace deep_emboss
}  // namespace koocadcam::skill
