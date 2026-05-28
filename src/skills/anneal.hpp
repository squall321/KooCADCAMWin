#pragma once
// @lat: [[engine/skills#anneal]]
//
// anneal — heat the workpiece above its recrystallization temperature,
// hold to homogenize, and slow-cool (in furnace, in ash, or in still air
// depending on alloy + section size).  The macroscopic GEOMETRY is
// unchanged; the MATERIAL changes:
//
//   - hardness drops (low / soft state)
//   - residual stress drops (relieved)
//   - grain coarsens (slow cool from above recrystallization temp)
//   - ductility / machinability increases
//
//        ┌─────────────┐                       ┌─────────────┐
//        │             │      anneal           │             │
//        │   stock     │  ────────────────▶   │   stock     │
//        │   (hard)    │  (T, t, slow-cool)    │   (soft)    │
//        │             │                       │             │
//        └─────────────┘                       └─────────────┘
//          shape identical;  hardness HB ↓;  σ_residual ↓
//
// Signature pattern:
//   - pattern.kind             = "anneal"
//   - pattern.material         = <input>
//   - pattern.temperature_c    = <input>
//   - pattern.hold_time_min    = <input>
//   - pattern.cooling_rate     = "slow_furnace" | "air"
//   - pattern.expected_hardness_hb (lookup)
//   - pattern.residual_stress  = "low"
//   - pattern.grain_size       = "coarse"
//   - pattern.geometry_changed = false
//
// DFM checks (all INFO-level):
//   DFM-HT-MATERIAL       material not in lookup table
//   DFM-HT-TEMP-LOW/HIGH  temperature outside material's typical anneal range
//   DFM-HT-COOLING        cooling_rate not "slow_furnace" or "air"
//   DFM-INPUT             hold_time_min ≤ 0 (this is "error")
//
// Synthesis:
//   NO geometric change.  Clone the workpiece (shape + material) and stamp
//   the signature.
//
// Recognize:
//   Impossible from B-rep alone (hardness / grain are sub-OCCT material
//   properties).  Returns ONLY signatures replayed from
//   `workpiece.features()` filtered by this skill_id — confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace anneal {

constexpr const char* kSkillId = "anneal";

struct Input
{
    std::string  material        = "carbon_steel_1045";
    double       temperature_c   = 800.0;
    double       hold_time_min   = 60.0;
    std::string  cooling_rate    = "slow_furnace";   // "slow_furnace" | "air"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace anneal
}  // namespace koocadcam::skill
