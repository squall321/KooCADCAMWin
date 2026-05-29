#pragma once
// @lat: [[engine/skills#cable_stay_tension]]
//
// cable_stay_tension — record a cable-stay or post-tensioning tendon
// stressing operation.  A cable identified by `cable_id` has a target
// `design_tension_kn` per design drawings; the actual `jack_force_kn`
// applied during stressing is logged for QA.  KooCADCAM records this
// as metadata — the cable body is sub-OCCT (separate from the deck B-rep).
//
//          ╲                                  ╱
//           ╲  cable_id, design_tension_kn   ╱
//            ╲       jack_force_kn          ╱
//             ╲                            ╱      cable stays
//          ──────────────────────────────────       (anchored at pylon + deck)
//                       deck
//
// Signature pattern:
//   - pattern.kind                = "cable_stay_tension"
//   - pattern.cable_id            = cable identifier (e.g. "S12N")
//   - pattern.design_tension_kn   = design target force (kN)
//   - pattern.jack_force_kn       = applied jack force (kN)
//   - pattern.tension_deviation_pct = (jack - design) / design × 100
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   cable_id           non-empty                    (DFM-INPUT error)
//   design_tension_kn  > 0                          (DFM-INPUT error)
//   jack_force_kn      > 0                          (DFM-INPUT error)
//   |dev| > 5%                                      (DFM-CABLE-TENSION error)
//   |dev| > 2%                                      (DFM-CABLE-TENSION info)
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

namespace cable_stay_tension {

constexpr const char* kSkillId = "cable_stay_tension";

struct Input
{
    std::string  cable_id           = "S1N";        // cable identifier
    double       design_tension_kn  = 5000.0;       // design force target (kN)
    double       jack_force_kn      = 5000.0;       // applied jack force (kN)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cable_stay_tension
}  // namespace koocadcam::skill
