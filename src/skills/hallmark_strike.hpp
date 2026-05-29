#pragma once
// @lat: [[engine/skills#hallmark_strike]]
//
// hallmark_strike — impress a statutory hallmark (e.g. 925, 750, 999)
// onto a finished jewelry piece using a hardened steel punch.  Marks
// alloy purity for compliance with assay-office regulations.  In
// reality this is a sub-millimetre indent (<200 µm deep) — captured
// here as metadata-only because the geometric impact on the host B-rep
// is below typical CAD modelling tolerance.
//
//        ┌─────────────┐                       ┌─────────────┐
//        │ finished    │   hallmark_strike     │ hallmarked  │
//        │ jewelry     │ ────────────────────▶│ jewelry     │
//        │             │ (hallmark_id, purity) │             │
//        └─────────────┘                       └─────────────┘
//
// Signature pattern:
//   pattern.kind             = "hallmark_strike"
//   pattern.hallmark_id      = string  ("UK-Assay-LON-925", "KS-D-2308-750", …)
//   pattern.alloy_purity     = int     (925 | 750 | 999 | 950 | …)
//   pattern.force_kn         = double  (press force in kN)
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             hallmark_id empty (error)
//   DFM-PURITY            alloy_purity ∉ {333, 375, 585, 750, 800, 925, 950, 990, 999} (error)
//   DFM-FORCE             force_kn ≤ 0 (error)
//   DFM-FORCE-LO          force_kn < 0.2 — likely under-impressed (info)
//   DFM-FORCE-HI          force_kn > 5.0 — punch-through risk on thin section (info)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hallmark_strike {

constexpr const char* kSkillId = "hallmark_strike";

struct Input
{
    std::string  hallmark_id  = "ASSAY-925";
    int          alloy_purity = 925;       // 333 | 375 | 585 | 750 | 800 | 925 | 950 | 990 | 999
    double       force_kn     = 1.0;       // press force in kN
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace hallmark_strike
}  // namespace koocadcam::skill
