#pragma once
// @lat: [[engine/skills#wood_drilling]]
//
// wood_drilling — drill a hole into a wooden workpiece using a brad-point,
// spade, Forstner, or auger bit.  Topologically identical to drill_hole
// (the result is a cylindrical bore) but the DFM envelope is wood-specific:
//   - generous diameter range (3 mm – 50 mm)
//   - depth / dia ratio relaxed (auger bits clear chips well)
//   - grain direction matters (end-grain WARN: tear-out risk)
//
//                ┌─ entry_face
//                ▼
//          ──────┬──────────┬──────
//                │   axis   │
//                │ ╱│╲      │       ↑ depth_mm
//                │╱ │ ╲     │       │
//                └──┴──┘            ↓
//                   ↑
//                  dia_mm
//
// Signature pattern (same as drill_hole):
//   - 1 cylindrical face (the bore wall)
//   - 2 circular edges (top + bottom OR top + exit)
//   - 1 planar bottom face IF blind
//   - bit_type recorded in pattern for downstream selection
//
// DFM checks:
//   DFM-WOOD-DIA-MIN   diameter_mm < 3 mm      (error — bit fragility)
//   DFM-WOOD-DIA-MAX   diameter_mm > 50 mm     (warn  — beyond Forstner range)
//   DFM-WOOD-DEPTH     depth/dia > 10          (warn  — auger needed for deep)
//   DFM-WOOD-ENDGRAIN  drilling end-grain      (info  — tear-out risk)
//   DFM-INPUT          depth_mm <= 0; diameter_mm <= 0
//
// Recognize:
//   Reuses the cylindrical-face + circular-edge pattern.  Distinguishes
//   from drill_hole purely by metadata; geometric recognition alone cannot
//   tell wood drilling apart from metal drilling — confidence is medium when
//   recovered from geometry, 1.0 when metadata replays.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wood_drilling {

constexpr const char* kSkillId = "wood_drilling";

struct Input
{
    FaceDatum    entry_face;
    double       position_x_mm = 0.0;
    double       position_y_mm = 0.0;
    gp_Dir       axis_dir      { 0.0, 0.0, -1.0 };
    double       diameter_mm   = 0.0;
    double       depth_mm      = 0.0;
    bool         through_hole  = false;
    std::string  bit_type      = "brad_point";   // "brad_point" | "spade" | "forstner" | "auger"
    bool         end_grain     = false;          // hint: drilling along grain end-cap
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wood_drilling
}  // namespace koocadcam::skill
