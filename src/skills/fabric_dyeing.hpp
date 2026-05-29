#pragma once
// @lat: [[engine/skills#fabric_dyeing]]
//
// fabric_dyeing — immerse fabric in a dye bath under controlled temperature
// and chemistry so that the dye is absorbed and fixed onto the substrate.
// No macroscopic geometry change; the colour, fixation %, and bath chemistry
// are recorded in the signature.  Common dye families:
//   - reactive    (cotton, cellulose)
//   - disperse    (polyester, hydrophobic synthetics)
//   - acid        (wool, silk, nylon — protein and nylon fibres)
//   - vat         (cotton — high washfastness)
//   - direct      (cotton — economical, lower fastness)
//
//        ┌─────────────────────────┐
//        │ ╔═════════════════════╗ │
//        │ ║   dye liquor T°C    ║ │  ── fabric immersed and circulated ──
//        │ ║   pH ~ recipe       ║ │
//        │ ╚═════════════════════╝ │
//        └─────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "fabric_dyeing"
//   - pattern.dye_type         = "reactive" | "disperse" | "acid" | "vat" | "direct"
//   - pattern.fixation_pct     = 0..100 (% of dye chemically bound to fibre)
//   - pattern.temp_c           = bath temperature (°C)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   fixation_pct ∈ [0, 100]       (DFM-INPUT          error)
//   temp_c > 0                    (DFM-INPUT          error)
//   dye_type recognised           (DFM-TEX-DYE        info)
//   fixation_pct ≥ 70             (DFM-TEX-FIXATION   info — fastness risk)
//   temp_c ∈ [20, 140]            (DFM-TEX-DYE-TEMP   info)
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

namespace fabric_dyeing {

constexpr const char* kSkillId = "fabric_dyeing";

struct Input
{
    std::string  dye_type      = "reactive";   // reactive | disperse | acid | vat | direct
    double       fixation_pct  = 85.0;         // % of dye fixed to fibre
    double       temp_c        = 60.0;         // dye-bath temperature
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fabric_dyeing
}  // namespace koocadcam::skill
