#pragma once
// @lat: [[engine/skills#nitride]]
//
// nitride — case-hardening by nitrogen diffusion.  Unlike carburize, the
// part is held below the critical temperature (typically 500 – 575 °C),
// so NO quench is required and dimensional distortion is minimal.
// Nitrogen-rich species (NH₃ for gas, N₂ glow for plasma, salt bath) drive
// nitrogen + (often) carbon into the surface to form hard nitrides.
//
//        ┌─────────────────────┐
//        │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │  ← compound layer (white layer, very hard)
//        │ ░░░░░░░░░░░░░░░░░░░ │  ← diffusion zone (nitrogen-strengthened)
//        │   (unchanged core)  │
//        └─────────────────────┘
//          shape identical;  surface HV ↑ (~ 1000 – 1300 HV)
//
// Signature pattern:
//   - pattern.kind            = "nitride"
//   - pattern.material        = <input>
//   - pattern.temperature_c   = <input>     (typ. 500 – 575 °C)
//   - pattern.time_h          = <input>     (typ. 10 – 80 h)
//   - pattern.case_depth_mm   = <input>     (typ. 0.05 – 0.6 mm)
//   - pattern.type            = "gas" | "plasma" | "salt_bath"
//   - pattern.geometry_changed = false
//   - pattern.requires_temper  = false   (no quench — no temper needed)
//
// DFM checks:
//   DFM-INPUT             temperature_c ≤ 0 / time_h ≤ 0       → error
//   DFM-NITRIDE-CASE      case_depth_mm ∉ [0.02, 1.0]          → error
//   DFM-NITRIDE-TYPE      type ∉ {gas, plasma, salt_bath}      → info
//   DFM-NITRIDE-TEMP      temperature_c ∉ [480, 600]           → info
//   DFM-NITRIDE-MATERIAL  non-nitridable family                → info
//
// Synthesis:
//   NO geometric change — clone with signature stamp.
//
// Recognize:
//   Metadata replay only — confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace nitride {

constexpr const char* kSkillId = "nitride";

struct Input
{
    std::string  material       = "alloy_steel_4140";
    double       temperature_c  = 525.0;   // typical 500 – 575 °C
    double       time_h         = 40.0;    // typical 10 – 80 h
    double       case_depth_mm  = 0.3;     // typical 0.05 – 0.6 mm
    std::string  type           = "gas";   // "gas" | "plasma" | "salt_bath"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace nitride
}  // namespace koocadcam::skill
