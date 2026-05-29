#pragma once
// @lat: [[engine/skills#battery_pack_assemble]]
//
// battery_pack_assemble — record assembly of N modules into an EV traction
// battery pack with a specified cooling strategy.  Macroscopic pack tray /
// enclosure B-rep is not changed by this skill; the modules and cooling
// plates are sub-OCCT bodies tracked only as metadata.  Used by MES / BOM /
// thermal-design verification.
//
//       ┌───────────────────────────────────────┐
//       │   ┌──┐ ┌──┐ ┌──┐ ┌──┐ ┌──┐ ┌──┐       │
//       │   │M0│ │M1│ │M2│ │M3│ │M4│ │M5│  …    │  ← module_count
//       │   └──┘ └──┘ └──┘ └──┘ └──┘ └──┘       │
//       │  ░░░░░░░  liquid coolant manifold ░░░ │  ← cooling_type
//       └───────────────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "battery_pack_assemble"
//   - pattern.module_count     = <input>
//   - pattern.total_kwh        = <input>
//   - pattern.cooling_type     = "liquid" | "air"
//   - pattern.geometry_changed = false
//
// DFM checks:
//   module_count ≥ 1                          (DFM-INPUT error)
//   total_kwh    > 0                          (DFM-INPUT error)
//   cooling_type ∈ {"liquid", "air"}          (DFM-PACK-COOL error)
//   kwh per module ≤ 12 (sanity)              (DFM-PACK-KWH info)
//
// Synthesis: NO geometric change. Clone + stamp.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace battery_pack_assemble {

constexpr const char* kSkillId = "battery_pack_assemble";

struct Input
{
    int          module_count = 8;        // 4–24 typical
    double       total_kwh    = 60.0;     // pack energy
    std::string  cooling_type = "liquid"; // "liquid" | "air"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace battery_pack_assemble
}  // namespace koocadcam::skill
