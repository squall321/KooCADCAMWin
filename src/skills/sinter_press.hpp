#pragma once
// @lat: [[engine/skills#sinter_press]]
//
// sinter_press — classical press-and-sinter powder metallurgy: uniaxial
// die-press a metal powder into a green compact, then sinter in a
// controlled atmosphere furnace below the alloy melting point.  Cheap,
// high volume, suited to simple net-shape parts (gears, bushings,
// structural brackets).
//
//      ┌──────────┐  uniaxial   ┌──────────┐  sinter   ┌──────────┐
//      │ loose    │  press      │  green   │  T < Tm   │ sintered │
//      │ powder   │  ──────▶    │  compact │  ──────▶  │  part    │
//      │ + lube   │  P (MPa)    │  ρ ≈ 85% │  hold (min)│ ρ ≈ 95% │
//      └──────────┘             └──────────┘           └──────────┘
//
// Model: workpiece IS the final sintered shape.  Process records the
// compaction pressure, sinter T + t recipe.  Slight shrinkage during
// sinter is typically pre-compensated in die design (factor < 3 % linear
// for press-and-sinter, vs ~ 20 % for MIM) — we do NOT scale geometry.
//
// DFM:
//   DFM-SP-PRESS      : compaction_pressure_mpa in [200, 800] envelope.
//   DFM-SP-SINTER-T   : sinter_temp_c in [700, 1400] envelope.
//   DFM-SP-SINTER-t   : sinter_time_min > 0 (error); info if outside [15, 240].
//   DFM-SP-MAT        : material in PM catalog (info on unknowns).
//
// Signature pattern:
//   pattern.kind                     == "sinter_press"
//   pattern.compaction_pressure_mpa, pattern.sinter_temp_c, pattern.sinter_time_min
//   pattern.expected_density_pct     (lookup-based)
//   pattern.process_family           == "powder_metallurgy"
//   pattern.geometric_change         == false
//
// Recognize:
//   Metadata-only — replays history if present.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sinter_press {

constexpr const char* kSkillId = "sinter_press";

struct Input
{
    std::string  material                  = "iron_fc_0205";
    double       compaction_pressure_mpa   = 600.0;
    double       sinter_temp_c             = 1120.0;
    double       sinter_time_min           = 45.0;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace sinter_press
}  // namespace koocadcam::skill
