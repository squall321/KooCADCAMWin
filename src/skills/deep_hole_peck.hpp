#pragma once
// @lat: [[engine/skills#deep_hole_peck]]
//
// deep_hole_peck — standard twist drill, but executed with a PECK cycle:
// the drill repeatedly retracts to evacuate chips, then advances by
// peck_depth_mm, until the full depth is reached.  Used for deep holes
// (depth/dia between 4 and 10) where chip evacuation matters but full
// gun-drilling is overkill.
//
// Geometrically identical to drill_hole.  Differentiated by:
//   - signature.pattern.kind       = "deep_hole_peck"
//   - tooling.peck_count           = ceil(depth / peck_depth)
//   - tooling.extra.peck_depth_mm
//
//                          ┌─ entry_face
//                          ▼
//                    ──────┬─────┬──────
//                          │     │  ▼ axis
//                          │  ↑  │       ↑ depth_mm (peck cycle)
//                          │  ↓ ↑│       │
//                          │  ↓ ↑│       │  (each ↕ pair = one peck)
//                          │  ↓  │       ↓
//                          └─────┘
//                           ↑
//                          dia_mm
//
// Signature pattern:
//   - 1 cylindrical face + 2 circular edges + 1 planar bottom
//   - signature.pattern.kind == "deep_hole_peck"
//   - signature.pattern.peck_depth_mm
//
// DFM checks:
//   DFM-002                : diameter_mm ≥ 0.8 mm
//   DFM-PECK-DIA           : diameter_mm ≥ 0.8 mm  (we tighten the floor on this skill)
//   DFM-PECK-DEPTH         : peck_depth ≥ dia × 0.5
//   DFM-PECK-RATIO         : depth/dia in [4, 10]  (sweet spot — outside warns)
//
// Recognize:
//   Cylindrical face with depth/dia in [4, 10] AND dia ≥ 0.8.  Confidence
//   0.5 — overlaps with drill_hole and gun_drill.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace deep_hole_peck {

constexpr const char* kSkillId = "deep_hole_peck";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm  = 0.0;
    double      position_y_mm  = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    double      diameter_mm    = 0.0;
    double      depth_mm       = 0.0;
    bool        through_hole   = false;

    // depth per peck.  Sentinel <=0 means "use 1.5 × diameter_mm" (auto).
    double      peck_depth_mm  = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace deep_hole_peck
}  // namespace koocadcam::skill
