#pragma once
// @lat: [[engine/skills#charging_port_install]]
//
// charging_port_install — record fitment of an EV charging inlet (CCS,
// CHAdeMO, Type 2 or NACS) into the pre-machined vehicle pocket.  The pocket
// itself was created by an upstream `mill_rect_pocket` + `drill_hole`
// sequence; this skill stamps the installed port record + its peak power
// rating so commissioning / homologation can verify the contactor and HV
// interlock match.
//
//           outer skin ────────────────╮
//        ┌──────────────────────────┐  │
//        │       ╱ ── ── ╲          │  │
//        │      │   ⊙ ⊙   │  ←──── port_type (CCS / CHAdeMO / Type2 / NACS)
//        │       ╲ ── ── ╱          │  │  + max_kw rating
//        └──────────────────────────┘  │
//
// Signature pattern:
//   - pattern.kind             = "charging_port_install"
//   - pattern.port_type        = "CCS" | "CHAdeMO" | "Type2" | "NACS"
//   - pattern.max_kw           = <input>
//   - pattern.geometry_changed = false
//
// DFM checks:
//   port_type ∈ {"CCS","CHAdeMO","Type2","NACS"}  (DFM-PORT-TYPE error)
//   max_kw > 0                                    (DFM-INPUT error)
//   max_kw ≤ rated upper for the chosen port      (DFM-PORT-KW error)
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

namespace charging_port_install {

constexpr const char* kSkillId = "charging_port_install";

struct Input
{
    std::string  port_type = "CCS";  // "CCS" | "CHAdeMO" | "Type2" | "NACS"
    double       max_kw    = 150.0;  // peak DC power
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace charging_port_install
}  // namespace koocadcam::skill
