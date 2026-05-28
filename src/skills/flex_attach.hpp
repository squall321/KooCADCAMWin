#pragma once
// @lat: [[engine/skills#flex_attach]]
//
// flex_attach — Flexible PCB (FPC) cable mating, either via ZIF (Zero-Insertion
// Force) connector or via ACF (Anisotropic Conductive Film) hot-bar bonding.
// The cable's exposed contacts mate to the host board with a defined contact
// count and retention force.  The macroscopic B-rep is UNCHANGED — connectors
// / ACF films are pre-existing components.  What is recorded is the join
// recipe: method, contact count, retention force.
//
//           ╔════════════════ flex cable ════════════════╗
//             ║▓▓▓▓▓▓▓▓ contacts  ▓▓▓▓▓▓▓▓║
//   ─────────╨───────────────────────────╨──────────────  ← host board / ZIF
//                                                          (retention_force_n)
//
// Signature pattern:
//   - pattern.kind              = "flex_attach"
//   - pattern.attach_method     = "ZIF" | "ACF" | "LIF" | "solder"
//   - pattern.contact_count     = <input>
//   - pattern.retention_force_n = <input>   measured pull-out force
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   DFM-INPUT              contact_count ≤ 0                  (error)
//   DFM-INPUT              retention_force_n ≤ 0              (error)
//   DFM-FLEX-METHOD        attach_method not in catalog       (error)
//   DFM-FLEX-FORCE-LOW     retention_force_n < 1.0 N          (error — falls out)
//   DFM-FLEX-FORCE-HIGH    retention_force_n > 80.0 N         (info — over-spec, tearing risk)
//   DFM-FLEX-PITCH         contact_count > 100 with ZIF       (info — sub-0.3 mm pitch territory)
//
// Recognize:
//   Impossible from B-rep.  Returns ONLY signatures replayed from
//   `workpiece.features()` filtered by skill_id — confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace flex_attach {

constexpr const char* kSkillId = "flex_attach";

struct Input
{
    std::string  attach_method      = "ZIF";   // "ZIF" | "ACF" | "LIF" | "solder"
    int          contact_count      = 0;
    double       retention_force_n  = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace flex_attach
}  // namespace koocadcam::skill
