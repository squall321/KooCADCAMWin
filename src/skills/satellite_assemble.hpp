#pragma once
// @lat: [[engine/skills#satellite_assemble]]
//
// satellite_assemble — clean-room integration record for a spacecraft bus.
// Captures the bus identifier, fully integrated dry mass (kg), and the
// number of instruments installed on the payload deck.  The bus B-rep is
// already correct from earlier structural fabrication skills; this skill
// stamps the integration metadata so AIT (assembly, integration, test)
// can downstream verify mass budget and payload manifest.
//
// Signature pattern:
//   - pattern.kind              = "satellite_assemble"
//   - pattern.bus_id            = bus serial identifier
//   - pattern.mass_kg           = integrated dry mass (kg)
//   - pattern.instrument_count  = number of payload instruments
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   bus_id            non-empty           (DFM-INPUT error)
//   mass_kg           ∈ (0, 25000]        (DFM-SPACE-MASS error if outside)
//   instrument_count  ∈ [1, 64]           (DFM-SPACE-PAYLOAD error if outside)
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

namespace satellite_assemble {

constexpr const char* kSkillId = "satellite_assemble";

struct Input
{
    std::string bus_id            = "BUS-0001";
    double      mass_kg           = 500.0;
    int         instrument_count  = 3;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace satellite_assemble
}  // namespace koocadcam::skill
