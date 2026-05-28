#pragma once
// @lat: [[engine/skills#ion_implant]]
//
// ion_implant — dopant introduction into silicon by accelerating ions in
// the keV range and embedding them at a depth controlled by ion energy.
// Macroscopic GEOMETRY is unchanged; what changes is the local electronic
// profile of the substrate (doping concentration vs depth).  Recorded as
// a METADATA-ONLY skill so the process plan can drive the implanter and
// downstream activation / drive-in anneals can be modelled.
//
//        ┌────────────┐                        ┌────────────┐
//        │  wafer     │      ion_implant       │  wafer +   │
//        │            │  ────────────────────▶ │  doped     │
//        └────────────┘  (dopant, E, dose, θ)  │  layer     │
//                                              └────────────┘
//             shape identical;  dopant profile changes
//
// Signature pattern:
//   - pattern.kind             = "ion_implant"
//   - pattern.dopant           = "B" | "P" | "As" | "Sb" | other
//   - pattern.energy_kev       = <input>
//   - pattern.dose_atoms_cm2   = <input>
//   - pattern.tilt_deg         = <input>      (0–60 typical)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   dopant non-empty
//   energy_kev     > 0  (typical 0.5 – 500 keV; outside is info)
//   dose_atoms_cm2 > 0
//   tilt_deg in [0, 60] (extreme tilts shadow features → info)
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ion_implant {

constexpr const char* kSkillId = "ion_implant";

struct Input
{
    std::string  dopant         = "B";        // B | P | As | Sb | …
    double       energy_kev     = 50.0;
    double       dose_atoms_cm2 = 1.0e15;
    double       tilt_deg       = 7.0;        // 7° is "channelling-suppress" classic
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ion_implant
}  // namespace koocadcam::skill
