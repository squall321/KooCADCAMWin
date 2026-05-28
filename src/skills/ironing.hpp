#pragma once
// @lat: [[engine/skills#ironing]]
//
// ironing — pass a previously drawn cup through smaller dies to thin the
// walls while keeping the outer diameter approximately constant (some
// sources say the outer diameter is preserved; others note slight outer
// shrinkage with significant inner enlargement).  We adopt the convention
// that the OUTER radius stays the same and the INNER radius grows, so the
// wall thickness decreases by the requested percentage.
//
//             ┌─┐                ┌──┐
//             │ │  (thick wall)  │  │  (thin wall)
//             │ │   →            │  │
//             │ │                │  │
//             └─┘                └──┘
//             original           after ironing
//
// Synthesis strategy (slice-1, robust):
//   1. Locate the cup's outer cylindrical wall (vertical axis cylinder
//      with the largest radius, spanning > sheet_thickness vertically).
//   2. The current inner radius can be inferred from the cup's existing
//      inner cylindrical face (if present); otherwise we assume the
//      pattern wall_thickness is the smallest bbox extent.
//   3. Enlarge the inner cavity by `wall_reduction_pct` × current_wall.
//      Implementation: cut a slightly larger inner cylinder from the cup.
//
// For workpieces that already have a `deep_draw` signature in their feature
// history, we read the original cup dimensions from there; otherwise we
// fall back to the geometric inference above.
//
// Signature pattern:
//   - 1 outer cylindrical wall (unchanged)
//   - 1 inner cylindrical wall (radius increased)
//   - wall_thickness reduced by `wall_reduction_pct`
//
// DFM checks:
//   DFM-IR-PASS       : wall_reduction_pct ≤ 50 (per pass industrial limit)
//   DFM-IR-MIN-WALL   : resulting wall ≥ 0.4 mm
//   DFM-INPUT         : wall_reduction_pct > 0
//
// Recognize:
//   Vertical-axis outer cylinder paired with a slightly smaller inner
//   cylindrical cavity, where the wall thickness is < sheet_thickness
//   (i.e. evidence of prior ironing).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ironing {

constexpr const char* kSkillId = "ironing";

struct Input
{
    // Assumed cup workpiece: a previously deep-drawn cup.  The cup centre
    // is inferred from the existing cylindrical wall topology; the caller
    // may optionally provide a hint (XY).  For slice-1 we use the largest
    // vertical-axis cylinder.
    double cup_position_x_hint_mm = 0.0;
    double cup_position_y_hint_mm = 0.0;
    bool   use_position_hint      = false;
    double wall_reduction_pct     = 30.0;  // (0, 50] per pass
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ironing
}  // namespace koocadcam::skill
