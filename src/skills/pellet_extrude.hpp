#pragma once
// @lat: [[engine/skills#pellet_extrude]]
//
// pellet_extrude — primary polymer compounding extrusion: solid resin
// pellets are fed into a heated single-screw barrel, melted, and pushed
// through a strand die to form continuous polymer rod (which a downstream
// pelletizer chops back into compounded pellets).  The MACROSCOPIC stock
// B-rep is unchanged — this skill captures the run record (polymer grade,
// throughput, screw speed) for process planning, not the strand geometry.
//
//   pellets ──▶ ┌────────────────────────┐
//               │   barrel + screw       │  ──▶  strand die  ──▶  pellets
//               │   T_melt, ω_screw_rpm  │
//               └────────────────────────┘
//
// Signature pattern:
//   - pattern.kind                = "pellet_extrude"
//   - pattern.polymer             = grade string (e.g. "ABS", "PP", "PA66")
//   - pattern.throughput_kg_h     = mass throughput
//   - pattern.screw_speed_rpm     = drive speed
//   - pattern.specific_energy_kwh_kg (estimate)
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   throughput_kg_h    ∈ (0, 2000]      DFM-EXT-THROUGHPUT
//   screw_speed_rpm    ∈ (0, 800]       DFM-EXT-SCREW
//   polymer            non-empty        DFM-INPUT
//
// Synthesis:
//   NO geometric change — clone shape + material, stamp signature.
//
// Recognize:
//   Metadata-only (pellet output is sub-OCCT).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pellet_extrude {

constexpr const char* kSkillId = "pellet_extrude";

struct Input
{
    std::string polymer            = "ABS";
    double      throughput_kg_h    = 100.0;
    double      screw_speed_rpm    = 200.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pellet_extrude
}  // namespace koocadcam::skill
