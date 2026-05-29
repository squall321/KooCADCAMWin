#pragma once
// @lat: [[engine/skills#open_die_forge]]
//
// open_die_forge — open-die (smith / hammer) forging.  Hot stock is placed
// between flat (or simple V- / swage-) dies and progressively reduced by
// repeated blows.  Because the dies do NOT enclose the workpiece, the final
// shape is constrained only by operator skill — this skill models the
// dominant effect: a thickness reduction along one axis with the lateral
// extents free to grow (volume preservation handled by callers).
//
//        ┌────────┐                            ┌──────────┐
//   ───► │ stock  │  flat dies + hammer  ───►  │  upset   │  ──►
//        └────────┘                            └──────────┘
//                  height ↓      footprint ↑ (free)
//
// Synthesis strategy:
//   Identify the thickness axis (smallest bbox extent by default; caller may
//   nudge via `reduction_pct`).  Rebuild the workpiece as a new cuboid with
//   the same other-two extents and a reduced thickness.  Lateral spread is
//   not modeled in slice-1 — open-die forging output is rarely a clean box
//   anyway, so callers using this for shape-critical work should switch to
//   `closed_die_forge`.
//
// Signature pattern:
//   - kind                = "open_die_forge"
//   - original_thickness  / target_thickness (mm)
//   - reduction_pct        (effective)
//   - thickness_axis      ("x" | "y" | "z")
//   - temp_c              (forging temperature)
//
// DFM checks:
//   DFM-INPUT          : reduction_pct ∈ (0, 100]; temp_c > 0
//   DFM-FORGE-COLD     : temp_c < 700 °C (steel cold-work envelope; info)
//   DFM-FORGE-OVER     : reduction_pct > 80 % per heat — error
//
// Recognize:
//   Metadata-driven (open-die forged stock looks like any cuboid).  Returns
//   the prior metadata entry with confidence 1.0; geometric fallback emits
//   a low-confidence candidate when smallest bbox extent < 20 mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace open_die_forge {

constexpr const char* kSkillId = "open_die_forge";

struct Input
{
    double  reduction_pct = 25.0;      // percent reduction of thickness axis
    double  temp_c        = 1100.0;    // forging temperature (hot working)
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace open_die_forge
}  // namespace koocadcam::skill
