#pragma once
// @lat: [[engine/skills#multi_step_bore]]
//
// multi_step_bore — a stepped bore with 3 or more diameter steps.  Extends
// bore_with_shelf (which has exactly 2 steps) to N coaxial cylinders, each
// narrower than the one above, separated by annular planar shoulders.
//
//                       ┌─ entry_face
//                       ▼
//                 ──────┬──────┬─────────────
//                       │step 0│              ↑ steps[0].depth
//                       │ wide │              ↓
//                ───────┴───┬──┴─────────── (shelf 0→1)
//                           │step 1          ↑
//                           │                ↓ steps[1].depth
//                       ────┴──┬─────── (shelf 1→2)
//                              │step 2       ↑
//                              │             ↓ steps[2].depth
//                              └─────── (blind bottom)
//
// Steps are ordered top→bottom; each step is strictly narrower than the
// previous one.  The first step is the entry diameter; the last step is the
// deepest, narrowest pocket.
//
// Signature pattern:
//   - N cylindrical faces with monotonically decreasing diameters
//   - (N-1) annular planar shelves between consecutive steps
//   - 1 planar bottom (the last step's blind floor)
//   - circular edges at the entry top, each shelf inner/outer, and the bottom
//   - signature.pattern.step_count == N
//   - signature.pattern.steps      == [{dia, depth}, …]
//
// DFM checks:
//   DFM-INPUT          : steps.size() ≥ 3 (else use bore_with_shelf)
//   DFM-002            : each step dia ≥ 0.8 mm
//   DFM-MSB-ORDER      : strictly decreasing diameters
//   DFM-MSB-DEPTH      : total depth fits in workpiece (warn if > 30 mm)
//
// Synthesis IMPORTANT:
//   OCCT's BRepAlgoAPI_Cut on overlapping CONCENTRIC cylinders MISSES the
//   interior overlap when the cylinders are packed into a single compound
//   tool (the compound's surfaces self-intersect).  We therefore:
//     1. Build N cylinders, each from its (z_start) to (z_end), of the
//        corresponding dia.  Each cylinder is positioned to overlap slightly
//        with the cylinder above it.
//     2. FUSE all N cylinders into a single solid first (fuseMany).
//     3. Then cut that single fused solid from the workpiece (single cut).
//
// Recognize:
//   N coaxial cylindrical faces with decreasing radii separated by annular
//   planar shoulders.  Confidence 0.85 (clean, well-defined topology).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace multi_step_bore {

constexpr const char* kSkillId = "multi_step_bore";

struct Step
{
    double dia_mm   = 0.0;
    double depth_mm = 0.0;
};

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };

    // Ordered top→bottom.  steps[0] = widest entry, steps[N-1] = deepest.
    // N must be ≥ 3 (else use bore_with_shelf).
    std::vector<Step> steps;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace multi_step_bore
}  // namespace koocadcam::skill
