#pragma once
// @lat: [[engine/skills#biomass_pelletize]]
//
// biomass_pelletize — extrusion of ground biomass (wood sawdust, straw,
// rice husk, miscanthus, …) through a die-and-roller pellet mill to
// produce compact cylindrical fuel pellets (ENplus / Pellet Fuels
// Institute spec).  The skill captures recipe metadata: source biomass,
// pellet diameter (6 mm or 8 mm are commercial defaults), and mill
// throughput.  It does not modify the source workpiece geometry — pellet
// mills consume a loose feedstock stream rather than a discrete B-rep
// part.
//
//      raw biomass ──▶ hammer mill ──▶ conditioner ──▶ pellet die
//                                                      │
//                                                      ▼
//                                              ◯◯◯  pellets
//                                              (dia_mm × ~3 dia_mm long)
//
// Signature pattern:
//   - pattern.kind             = "biomass_pelletize"
//   - pattern.biomass_type
//   - pattern.pellet_dia_mm    (6.0 or 8.0 typical)
//   - pattern.throughput_kg_h
//   - pattern.geometry_changed = false
//
// DFM checks:
//   biomass_type non-empty                       (DFM-INPUT warning)
//   biomass_type in known set                    (DFM-PELLET-FEED info)
//   pellet_dia_mm > 0                            (DFM-INPUT error)
//   pellet_dia_mm ∈ [4, 14]                      (DFM-PELLET-DIA info — ENplus 6/8)
//   throughput_kg_h > 0                          (DFM-INPUT error)
//   throughput_kg_h ≤ 10000                      (DFM-PELLET-RATE info — very large mill)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
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

namespace biomass_pelletize {

constexpr const char* kSkillId = "biomass_pelletize";

struct Input
{
    std::string  biomass_type    = "wood_sawdust";   // "wood_sawdust" | "straw" | "rice_husk" | "miscanthus"
    double       pellet_dia_mm   = 6.0;              // ENplus class spec: 6 mm or 8 mm
    double       throughput_kg_h = 1000.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace biomass_pelletize
}  // namespace koocadcam::skill
