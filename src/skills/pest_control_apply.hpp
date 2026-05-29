#pragma once
// @lat: [[engine/skills#pest_control_apply]]
//
// pest_control_apply — agricultural pesticide / pest-control chemical
// application record.  Captures chemical name, dose (millilitres per
// hectare), and application method (spray / drench / granule).  No
// geometric change.
//
//          ╔════════════════════════════════════════════════════════╗
//          ║                                                        ║
//          ║   ─── spray:    boom    ◌◌◌◌◌◌◌◌◌◌◌◌◌◌  ← fine mist     ║
//          ║                          ↓ ↓ ↓ ↓ ↓ ↓                   ║
//          ║   ─── drench:   tank → soil saturation                 ║
//          ║   ─── granule:  broadcaster → solid grains             ║
//          ║                                                        ║
//          ║   dose_ml_ha × area = total chemical                   ║
//          ║                                                        ║
//          ╚════════════════════════════════════════════════════════╝
//
// Signature pattern:
//   - pattern.kind               = "pest_control_apply"
//   - pattern.chemical_name      = string (regulated trade or active)
//   - pattern.dose_ml_ha
//   - pattern.application_method = "spray" | "drench" | "granule"
//   - pattern.geometry_changed   = false
//
// DFM checks:
//   chemical_name       non-empty                   (DFM-INPUT error)
//   dose_ml_ha          > 0                         (DFM-INPUT error)
//   dose_ml_ha          ≤ 10000                     (DFM-PEST-DOSE error if higher —
//                                                    exceeds typical regulatory cap)
//   application_method  ∈ {spray, drench, granule}  (DFM-PEST-METHOD info if unknown)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
//
// Recognize:
//   Metadata-only — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pest_control_apply {

constexpr const char* kSkillId = "pest_control_apply";

struct Input
{
    std::string  chemical_name      = "glyphosate";
    double       dose_ml_ha         = 1500.0;
    std::string  application_method = "spray";   // "spray" | "drench" | "granule"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pest_control_apply
}  // namespace koocadcam::skill
