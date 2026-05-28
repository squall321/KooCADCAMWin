#pragma once
// @lat: [[engine/skills#hot_isostatic_press]]
//
// hot_isostatic_press (HIP) — simultaneous high temperature and isostatic
// inert-gas pressure (typically argon at 100-200 MPa, 1000-1250 °C) closes
// internal porosity in cast, sintered, or AM parts, lifting density toward
// 100 % theoretical and improving fatigue life dramatically.
//
//      ┌─────────────────┐                           ┌─────────────────┐
//      │ porous part     │   T, P, t                 │ dense part      │
//      │ (cast / AM / PM)│   ─────────────────────▶  │ ρ ≈ 100 %       │
//      │ ρ ≈ 92-98 %     │   isostatic pressure      │ pores collapsed │
//      └─────────────────┘   no shape change         └─────────────────┘
//
// Model: pure microstructure modifier — densification only, no shape
// change at the outer envelope (small uniform shrinkage from pore closure
// is < 1 % and ignored in CAD).  Treated as a no-op in geometry,
// metadata-only in signature, like the heat-treatment family.
//
// DFM:
//   DFM-HIP-TEMP      : temperature_c in [800, 2000] envelope (typical
//                       industrial HIP range 1000-1250 for steels).
//   DFM-HIP-PRESS     : pressure_mpa in [50, 300] envelope (typical 100-200).
//   DFM-HIP-HOLD      : hold_min must be > 0 (rejection); info if < 30 or > 480.
//   DFM-HIP-MAT       : material in PM/cast catalog (info on unknowns).
//
// Signature pattern:
//   pattern.kind             == "hot_isostatic_press"
//   pattern.temp_c, pattern.pressure_mpa, pattern.hold_min
//   pattern.expected_density_pct (post-HIP, lookup-based)
//   pattern.process_family   == "powder_metallurgy"
//   pattern.geometric_change == false
//
// Recognize:
//   Metadata-only — internal porosity reduction is invisible to B-rep.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hot_isostatic_press {

constexpr const char* kSkillId = "hot_isostatic_press";

struct Input
{
    std::string  material      = "stainless_316";
    double       temp_c        = 1150.0;
    double       pressure_mpa  = 150.0;
    double       hold_min      = 240.0;       // 4 h typical
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace hot_isostatic_press
}  // namespace koocadcam::skill
