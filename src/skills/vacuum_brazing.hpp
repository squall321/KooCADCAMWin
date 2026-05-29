#pragma once
// @lat: [[engine/skills#vacuum_brazing]]
//
// vacuum_brazing — join two parts in a vacuum furnace by melting a thin
// filler metal between them.  The vacuum (≤ 1e-3 torr typical) prevents
// oxide formation and flux residue; the parent metals stay below their
// solidus while the filler wets the joint by capillary action.  Used for
// honeycomb panels, RF cavities, satellite waveguides, leak-tight
// stainless heat-exchanger laminates, and beryllium / titanium assemblies.
//
//      ┌─────── parent A ───────┐
//      ├─── filler (molten) ────┤   ← filler liquidus < peak_temp_c < parent solidus
//      └─────── parent B ───────┘
//          ambient = vacuum_torr (≤ 1e-3 typical)
//
// Signature pattern:
//   - pattern.kind             = "vacuum_brazing"
//   - pattern.filler_metal     = <input>
//   - pattern.vacuum_torr      = <input>
//   - pattern.peak_temp_c      = <input>
//   - pattern.joint_quality    = "leak_tight"
//   - geometry_changed         = false
//
// DFM checks:
//   filler_metal  non-empty                    (DFM-INPUT error)
//   vacuum_torr   > 0, ≤ 1e-3                  (DFM-BRAZE-VAC warning if > 1e-3)
//   peak_temp_c   ∈ (600, 1300]                (DFM-BRAZE-TEMP error otherwise)
//
// Synthesis: clone shape + material; stamp signature.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace vacuum_brazing {

constexpr const char* kSkillId = "vacuum_brazing";

struct Input
{
    std::string filler_metal  = "BAg-8";   // 72Ag/28Cu eutectic, common UHV
    double      vacuum_torr   = 1.0e-5;
    double      peak_temp_c   = 815.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace vacuum_brazing
}  // namespace koocadcam::skill
