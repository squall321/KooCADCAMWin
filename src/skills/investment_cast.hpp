#pragma once
// @lat: [[engine/skills#investment_cast]]
//
// investment_cast — lost-wax investment casting process for fine-detail
// metal parts.  A wax pattern of the part is dipped in ceramic slurry to
// build a shell; the wax is melted out and molten metal is poured in.
//
//      Step 1: wax pattern    Step 2: ceramic shell   Step 3: dewax + pour
//                              (multiple coats)
//        ┌───────────┐                                  ┌───────────┐
//        │░░░░░░░░░░░│           ┌▓▓▓▓▓▓▓▓▓▓▓▓▓┐         │█████████████│
//        │░░░ wax ░░░│   ──►     │▓░░░ wax ░░▓│   ──►   │█  metal  █│
//        │░░░░░░░░░░░│           │▓░░░░░░░░░░▓│         │█████████████│
//        └───────────┘           └▓▓▓▓▓▓▓▓▓▓▓▓▓┘         └───────────┘
//                                  ceramic shell           final cast
//
// vs sand_cast: investment casting can hold MUCH finer detail (≥ 1.5 mm
// walls) because the shell is rigid and reproduces every wax-pattern
// feature, but it shrinks more during cooling (wax + metal both shrink) so
// `wax_pattern_shrink_pct` must be applied UPSTREAM by the pattern maker.
//
// Model: same as sand_cast — the input workpiece IS the final part shape.
// We only validate and record process metadata.
//
// DFM:
//   DFM-CAST-WALL    : wall_thickness ≥ 1.5 mm (investment cast minimum —
//                      MUCH finer than sand cast's 3 mm).
//   DFM-CAST-SHRINK  : wax_pattern_shrink_pct ∈ [0.5, 3.0] (typical
//                      investment-cast shrink allowance).
//   DFM-CAST-MAT     : info — material catalog check.
//
// Signature pattern:
//   pattern.kind == "investment_cast"
//   pattern.material, pattern.wall_thickness_mm, pattern.wax_pattern_shrink_pct
//   pattern.geometric_change == false
//
// Recognize: metadata-only (same rationale as sand_cast).

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace investment_cast {

constexpr const char* kSkillId = "investment_cast";

struct Input
{
    std::string   material                  = "stainless_steel";
    double        wall_thickness_mm         = 0.0;
    double        wax_pattern_shrink_pct    = 1.5;  // typical 0.5–3.0 %
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace investment_cast
}  // namespace koocadcam::skill
