#pragma once
// @lat: [[engine/skills#fabric_finishing]]
//
// fabric_finishing — apply a mechanical and/or thermal finishing process to
// a dyed fabric to lock in dimensions, hand, surface lustre, or specific
// performance attributes (water-repellent, flame-retardant, antimicrobial,
// etc.).  Common techniques:
//   - sanforize    (controlled compressive shrink — prevents post-laundry shrinkage)
//   - calendering  (heated-roll surface lustre / smoothness)
//   - mercerize    (caustic-tension treatment for cotton — improves lustre + dye uptake)
//   - heatset      (synthetic-fibre dimensional stabilisation, ~170–210 °C)
//   - raising      (brushed nap finish — flannel / fleece)
//
// Metadata-only at the OCCT level: dimensional and surface effects are
// recorded in the signature.
//
// Signature pattern:
//   - pattern.kind             = "fabric_finishing"
//   - pattern.technique        = "sanforize" | "calendering" | "mercerize" | "heatset" | "raising"
//   - pattern.finishing_temp_c = process temperature (°C)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   finishing_temp_c > 0          (DFM-INPUT             error)
//   technique recognised          (DFM-TEX-FINISH        info)
//   finishing_temp_c ≤ 230        (DFM-TEX-FINISH-TEMP   info — fibre damage risk)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
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

namespace fabric_finishing {

constexpr const char* kSkillId = "fabric_finishing";

struct Input
{
    std::string  technique         = "sanforize";   // sanforize | calendering | mercerize | heatset | raising
    double       finishing_temp_c  = 120.0;         // process temperature (°C)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fabric_finishing
}  // namespace koocadcam::skill
