#pragma once
// @lat: [[engine/skills#wind_blade_layup]]
//
// wind_blade_layup — composite layup record for a wind-turbine blade.
// Multiple fiber plies (glass / carbon) are draped into the mold,
// infused with resin, and cured to form the half-shell.  The
// macroscopic mold-line surface is unchanged at the OCCT level; this
// skill stamps ply count / fiber type / blade length so downstream
// process planning can size cure cycle and resin volume.
//
//   root ──────────────── span (length_m) ──────────────── tip
//      ply 1 ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
//      ply 2 ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
//      …
//      ply N ▒▒▒▒▒▒▒▒
//
// Signature pattern:
//   - pattern.kind             = "wind_blade_layup"
//   - pattern.ply_count        = number of plies
//   - pattern.fiber_type       = "E-glass" | "S-glass" | "carbon"
//   - pattern.length_m         = blade span
//   - pattern.geometry_changed = false
//
// DFM checks:
//   ply_count  > 0                       (DFM-INPUT error)
//   length_m   ∈ [1.0, 120.0]            (DFM-BLADE-LEN error if outside)
//   fiber_type ∈ {E-glass,S-glass,carbon} (DFM-BLADE-FIBER info if unknown)
//
// Synthesis: no geometric change — clone + stamp.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wind_blade_layup {

constexpr const char* kSkillId = "wind_blade_layup";

struct Input
{
    int         ply_count  = 20;
    std::string fiber_type = "E-glass";   // "E-glass" | "S-glass" | "carbon"
    double      length_m   = 45.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wind_blade_layup
}  // namespace koocadcam::skill
