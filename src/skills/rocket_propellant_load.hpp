#pragma once
// @lat: [[engine/skills#rocket_propellant_load]]
//
// rocket_propellant_load — cryogenic / storable propellant loading record
// for a rocket stage.  Distinct from `propellant_load` (small-arms powder
// charge), this skill captures bulk LOX / RP-1 / LH2 / N2O4 / UDMH transfer
// metadata: chemical type, loaded mass (kg), and loading temperature (°C).
// The stage tank B-rep is unchanged — propellant is sub-OCCT bulk fluid
// inside the tank, so this skill stamps the propulsion charge record.
//
// Signature pattern:
//   - pattern.kind             = "rocket_propellant_load"
//   - pattern.propellant_type  = "LOX" | "RP-1" | "LH2" | "N2O4" | "UDMH"
//   - pattern.mass_kg          = loaded propellant mass (kg)
//   - pattern.load_temp_c      = bulk temperature at load complete (°C)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   propellant_type ∈ allow-list      (DFM-PROP-TYPE error if unknown)
//   mass_kg         ∈ (0, 5000000]    (DFM-PROP-MASS error if outside)
//   load_temp_c     plausible window per propellant
//                                     (DFM-PROP-TEMP error if outside)
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

namespace rocket_propellant_load {

constexpr const char* kSkillId = "rocket_propellant_load";

struct Input
{
    std::string propellant_type = "LOX";    // "LOX"|"RP-1"|"LH2"|"N2O4"|"UDMH"
    double      mass_kg         = 1000.0;
    double      load_temp_c     = -183.0;   // boiling point of LOX
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rocket_propellant_load
}  // namespace koocadcam::skill
