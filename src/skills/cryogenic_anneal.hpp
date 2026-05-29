#pragma once
// @lat: [[engine/skills#cryogenic_anneal]]
//
// cryogenic_anneal — soak the workpiece at deep-cryogenic temperatures
// (typically liquid-nitrogen, 77 K, or lower with LHe gradients) for an
// extended hold, then ramp back to ambient at a controlled rate.  Used to
// stabilize retained austenite in tool steels, relieve micro-residual
// stress in precision-machined parts, and condition superconducting
// fixtures.  The macroscopic GEOMETRY is unchanged; the MATERIAL state
// changes:
//
//   - retained austenite → fine martensite (tool steels)
//   - residual stress drops further than warm temper alone
//   - dimensional stability improves (no later γ→α transition)
//   - wear life improves 2–5× in many tool-steel applications
//
//       T (K)
//         300 ──┐                        ┌──── 300
//               │  ramp_k_min            │
//               ╲                        ╱
//                ╲                      ╱
//                 ╲                    ╱
//                  ╲                  ╱
//                   ╲ cold_temp_k    ╱
//                    └─── hold_h ───┘
//
// Signature pattern:
//   - pattern.kind             = "cryogenic_anneal"
//   - pattern.cold_temp_k      = <input>
//   - pattern.hold_h           = <input>
//   - pattern.ramp_k_min       = <input>
//   - pattern.thermal_cycle    = "ambient -> cold -> ambient"
//   - geometry_changed         = false
//
// DFM checks:
//   cold_temp_k  ∈ (0, 200]           (DFM-CRYO-TEMP error otherwise)
//   hold_h       > 0                  (DFM-INPUT error)
//   ramp_k_min   > 0, ≤ 5 K/min       (DFM-CRYO-RAMP warning if faster — thermal shock risk)
//
// Synthesis: clone shape + material verbatim; stamp signature.
// Recognize: metadata-only (sub-OCCT material state).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cryogenic_anneal {

constexpr const char* kSkillId = "cryogenic_anneal";

struct Input
{
    double cold_temp_k = 77.0;   // LN2 default
    double hold_h      = 24.0;
    double ramp_k_min  = 1.0;    // K per minute
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cryogenic_anneal
}  // namespace koocadcam::skill
