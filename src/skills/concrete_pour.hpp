#pragma once
// @lat: [[engine/skills#concrete_pour]]
//
// concrete_pour — place and finish a volume of fresh concrete into the
// formwork.  The poured volume, mix design (e.g. C25/30, C40/50 per
// EN 206), and slump are the three QA parameters captured per pour.
// KooCADCAM phase-1 records this as metadata; the cured shape is the
// formwork model, so the workpiece B-rep itself is unchanged.
//
//             ╔═══════════════════════════╗
//             ║░░░░░░░░░░░░░░░░░░░░░░░░░░░║   ← fresh concrete
//             ║░░░░░░░░░░░░░░░░░░░░░░░░░░░║      volume_m3, mix_design,
//             ║░░░░░░░░░░░░░░░░░░░░░░░░░░░║      slump_mm
//             ╚═══════════════════════════╝
//                       formwork
//
// Signature pattern:
//   - pattern.kind             = "concrete_pour"
//   - pattern.volume_m3        = poured volume
//   - pattern.mix_design       = mix code (e.g. "C25/30", "C40/50")
//   - pattern.slump_mm         = measured slump
//   - pattern.geometry_changed = false
//
// DFM checks:
//   volume_m3   > 0                            (DFM-INPUT error)
//   mix_design  non-empty                      (DFM-INPUT error)
//   slump_mm    ∈ [0, 250]                     (DFM-CONC-SLUMP error)
//   volume_m3   > 100  ⇒ stage pour            (DFM-CONC-VOL info)
//   slump_mm    > 220  ⇒ flowable/SCC          (DFM-CONC-SLUMP info)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace concrete_pour {

constexpr const char* kSkillId = "concrete_pour";

struct Input
{
    double       volume_m3   = 1.0;       // poured volume (m3)
    std::string  mix_design  = "C25/30";  // EN 206 mix code
    double       slump_mm    = 100.0;     // measured slump [0, 250]
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace concrete_pour
}  // namespace koocadcam::skill
