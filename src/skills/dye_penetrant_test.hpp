#pragma once
// @lat: [[engine/skills#dye_penetrant_test]]
//
// dye_penetrant_test (PT / LPI) — INSPECTION skill: capillary penetrant
// surface-crack detection.  Apply low-viscosity dye, dwell, wipe excess,
// apply developer; cracks bleed back visible (red) or fluorescent
// indications.  Geometrically a NO-OP.
//
//      penetrant applied         wipe + developer       indication
//       ───────────────             ───────────         ───●●●─────
//                                                         ↑
//                                                         crack bleed-out
//
// Works on ANY non-porous material (ferrous + non-ferrous + polymers).
//
// What we record:
//   - target face datum
//   - dwell_time_min        — penetrant dwell (5-30 min typical)
//   - sensitivity_class     — "level_1_low" | "level_2_med" | "level_3_high" |
//                             "level_4_ultra"
//
// DFM checks:
//   target face datum resolves
//   dwell_time_min > 0 (error)
//   dwell_time_min < 5  → info (under-soak; small cracks may miss)
//   dwell_time_min > 60 → info (over-soak; excess penetrant entrapment)
//   sensitivity_class is one of the four standard levels
//
// Material note:
//   Porous materials (sintered, some castings, foamed polymers) will
//   absorb penetrant uniformly and obscure indications — emit info.
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

namespace dye_penetrant_test {

constexpr const char* kSkillId = "dye_penetrant_test";

struct Input
{
    FaceDatum     target_face_datum;
    double        dwell_time_min      = 10.0;
    std::string   sensitivity_class   = "level_2_med";   // see header
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace dye_penetrant_test
}  // namespace koocadcam::skill
