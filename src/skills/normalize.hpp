#pragma once
// @lat: [[engine/skills#normalize]]
//
// normalize — heat the workpiece above the upper critical (A_c3) and hold,
// then cool in STILL AIR.  Cooling is faster than a furnace anneal but
// slower than a quench, producing a fine-grained, homogeneous
// microstructure (ferrite + pearlite for low-carbon, pearlite for medium-
// carbon).  Hardness sits BETWEEN annealed and quenched values; residual
// stress is low; grain size is FINE (the key differentiator from anneal).
//
//        ┌─────────────┐                       ┌─────────────┐
//        │             │  normalize            │ ▒▒▒▒▒▒▒▒▒▒▒ │
//        │   stock     │  (T, t, air-cool)     │  uniform    │
//        │             │  ─────────────────▶  │  fine grain │
//        │             │                       │             │
//        └─────────────┘                       └─────────────┘
//          shape identical;  grain size: FINE;  σ_residual: low
//
// Signature pattern:
//   - pattern.kind                  = "normalize"
//   - pattern.material              = <input>
//   - pattern.normalize_temperature_c = <input>
//   - pattern.hold_time_min         = <input>
//   - pattern.cooling_rate          = "air"  (always — defining feature)
//   - pattern.expected_grain_size   = "fine"
//   - pattern.residual_stress       = "low"
//   - pattern.geometry_changed      = false
//
// DFM checks (info-level except DFM-INPUT):
//   DFM-INPUT             temperature_c ≤ 0 / hold_time_min ≤ 0   → error
//   DFM-HT-MATERIAL       material not in lookup table            → info
//   DFM-HT-TEMP-*         outside material's normalize range      → info
//   DFM-HT-NA             material has no normalize range
//                         (e.g. aluminium / brass — no allotropic
//                         transformation)                          → info
//
// Synthesis:
//   NO geometric change.
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

namespace normalize {

constexpr const char* kSkillId = "normalize";

struct Input
{
    std::string  material                 = "carbon_steel_1045";
    double       normalize_temperature_c  = 870.0;
    double       hold_time_min            = 30.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace normalize
}  // namespace koocadcam::skill
