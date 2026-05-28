#pragma once
// @lat: [[engine/skills#micro_drill]]
//
// micro_drill — drilling sub-millimetre holes (0.1 mm ≤ dia < 0.8 mm).
// Requires a special carbide micro-drill, very high spindle RPM, very low
// feed, and very strict depth/dia limits (≤ 5) due to the stiffness wall.
//
//                       ┌─ entry_face
//                       ▼
//                 ──────┬─┬──────
//                       │ │  ▼ axis
//                       │ │           ↑ depth_mm (shallow — ≤ dia × 5)
//                       │ │           ↓
//                       └─┘
//                        ↑
//                       dia_mm  (very small, < 0.8 mm)
//
// Signature pattern:
//   - 1 cylindrical face + 2 circular edges + 1 planar bottom
//   - signature.pattern.kind == "micro_drill"
//   - signature.pattern.dia_mm
//
// DFM checks:
//   DFM-MICRO-DIAMIN  : dia ≥ 0.1 mm
//   DFM-MICRO-DIAMAX  : dia < 0.8 mm   (otherwise use drill_hole; warns ≥ 0.8)
//   DFM-MICRO-RATIO   : depth/dia ≤ 5  (carbide micro-drill stiffness limit)
//
// Recognize:
//   Cylindrical face with diameter < 0.8 mm.  Confidence 0.85 — strong
//   differentiation from drill_hole because the diameter envelope does not
//   overlap.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace micro_drill {

constexpr const char* kSkillId = "micro_drill";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      diameter_mm   = 0.0;            // ≥ 0.1 mm (sub-mm regime)
    double      depth_mm      = 0.0;
    bool        through_hole  = false;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace micro_drill
}  // namespace koocadcam::skill
