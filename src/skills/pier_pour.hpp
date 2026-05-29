#pragma once
// @lat: [[engine/skills#pier_pour]]
//
// pier_pour — record a single bridge pier monolithic concrete pour.
// A pier is identified by `pier_id` (e.g. "P3"), poured with a structural
// concrete `mix_grade` (e.g. "C40/50"), and the placed `volume_m3` is the
// QA hand-off to the cure-time / strength-gain schedule.  KooCADCAM
// records this as metadata; the cured pier geometry lives in the formwork
// model upstream, so the workpiece B-rep itself is unchanged.
//
//             ╔═══════════╗
//             ║░░░░░░░░░░░║   ← pier shaft (formwork cured)
//             ║░░░░░░░░░░░║      pier_id, volume_m3, mix_grade
//             ║░░░░░░░░░░░║
//             ╚═══════════╝
//                  ║ ║
//             ════╩═╩════         pile cap / footing
//
// Signature pattern:
//   - pattern.kind             = "pier_pour"
//   - pattern.pier_id          = pier identifier (free text, e.g. "P3")
//   - pattern.volume_m3        = poured volume
//   - pattern.mix_grade        = EN 206 / equivalent mix code
//   - pattern.geometry_changed = false
//
// DFM checks:
//   pier_id    non-empty                           (DFM-INPUT error)
//   volume_m3  > 0                                 (DFM-INPUT error)
//   mix_grade  non-empty                           (DFM-INPUT error)
//   volume_m3  > 80   ⇒ stage pour / temp control  (DFM-PIER-VOL info)
//   volume_m3 < 1.0   ⇒ unusually small pier       (DFM-PIER-VOL info)
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

namespace pier_pour {

constexpr const char* kSkillId = "pier_pour";

struct Input
{
    std::string  pier_id    = "P1";       // pier identifier
    double       volume_m3  = 25.0;       // poured volume (m3)
    std::string  mix_grade  = "C40/50";   // structural mix grade
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pier_pour
}  // namespace koocadcam::skill
