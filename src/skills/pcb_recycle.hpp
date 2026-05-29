#pragma once
// @lat: [[engine/skills#pcb_recycle]]
//
// pcb_recycle — end-of-life printed circuit board recovery.  Shredded PCB
// scrap is mechanically separated and pyro- or hydro-metallurgically
// processed to recover copper foil/traces and precious metals (Au, Ag, Pd)
// from solder pads and edge connectors.  Pure metadata operation: workpiece
// B-rep is unchanged; a recovery record is stamped for downstream materials
// accounting and producer-responsibility reporting.
//
//          ┌────────────────┐    pcb_recycle      ┌────────────────┐
//          │ EOL PCB scrap  │  ─────────────────▶ │ stamp record   │
//          │ pcb_mass_kg    │ (Cu%, PM g/t)       │ Cu + PM yields │
//          └────────────────┘                     └────────────────┘
//
// Signature pattern:
//   - pattern.kind                    = "pcb_recycle"
//   - pattern.pcb_mass_kg
//   - pattern.copper_yield_pct
//   - pattern.precious_metal_yield_g_t
//   - pattern.recovered_copper_kg     = pcb_mass_kg * copper_yield_pct / 100
//   - pattern.recovered_pm_g          = pcb_mass_kg * pm_yield_g_t / 1000
//   - pattern.geometry_changed        = false
//
// DFM checks:
//   pcb_mass_kg              > 0                  (DFM-INPUT error)
//   copper_yield_pct         ∈ (0, 100]           (DFM-INPUT error)
//   copper_yield_pct         ≤ 30 typical PCB     (DFM-PCB-CU info if exceeded)
//   precious_metal_yield_g_t ≥ 0                  (DFM-INPUT error if negative)
//   precious_metal_yield_g_t ≤ 1000 typical EOL   (DFM-PCB-PM info if exceeded)
//
// Synthesis: NO geometric change.  Clone + stamp signature.
// Recognize: Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pcb_recycle {

constexpr const char* kSkillId = "pcb_recycle";

struct Input
{
    double  pcb_mass_kg              = 5.0;      // shredded PCB scrap mass
    double  copper_yield_pct         = 20.0;     // recovered copper fraction
    double  precious_metal_yield_g_t = 250.0;    // recovered Au+Ag+Pd per tonne
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pcb_recycle
}  // namespace koocadcam::skill
