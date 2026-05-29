#pragma once
// @lat: [[engine/skills#propellant_load]]
//
// propellant_load — record charge of smokeless propellant into a primed case
// for authorized civilian/commercial small-arms manufacturing.  The case
// B-rep does not change (the powder is sub-OCCT bulk material inside the
// case).  This skill stamps the propellant lot + charge metadata so the
// batch record + lot traceability can downstream verify the load.
//
// Signature pattern:
//   - pattern.kind            = "propellant_load"
//   - pattern.propellant_type = "ball" | "stick" | "flake" | <product name>
//   - pattern.charge_grain    = powder mass (grain)
//   - pattern.lot_id          = supplier lot identifier (traceability)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   propellant_type  non-empty          (DFM-INPUT error)
//   charge_grain     ∈ (0, 200]         (DFM-AMMO-CHARGE error if outside)
//   lot_id           non-empty          (DFM-AMMO-LOT error)
//
// Synthesis: metadata-only.
// Recognize: metadata replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace propellant_load {

constexpr const char* kSkillId = "propellant_load";

struct Input
{
    std::string propellant_type = "ball";
    double      charge_grain    = 6.0;
    std::string lot_id          = "LOT-0000";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace propellant_load
}  // namespace koocadcam::skill
