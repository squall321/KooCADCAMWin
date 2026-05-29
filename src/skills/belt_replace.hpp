#pragma once
// @lat: [[engine/skills#belt_replace]]
//
// belt_replace — maintenance record for swapping a worn power-transmission
// belt (V-belt, timing/synchronous, or flat).  The belt itself is a
// sub-OCCT flexible body (not B-rep); this skill captures the
// installation + the static tension reading so vibration analytics can
// tie a downstream baseline shift to a documented belt swap.
//
//             ╔═══════╗            ╔═══════╗
//             ║       ║            ║       ║
//             ║   ●───╫────────────╫───●   ║
//             ║       ║   ════════ ║       ║
//             ║       ║            ║       ║
//             ╚═══════╝            ╚═══════╝
//               drive                 driven
//                  ↑ belt span, tension_n
//
// Signature pattern:
//   - pattern.kind             = "belt_replace"
//   - pattern.belt_type        = "V" | "timing" | "flat"
//   - pattern.belt_id          = catalog PN (e.g. "B-44", "HTD-8M-1280")
//   - pattern.tension_n        = static tension (N)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   belt_id   non-empty                       (DFM-INPUT error)
//   belt_type ∈ {V, timing, flat}             (DFM-BELT-TYPE info if unknown)
//   tension_n > 0                             (DFM-INPUT error if ≤ 0)
//   tension_n ∈ [50, 5000] N                  (DFM-BELT-TENSION info if outside)
//
// Synthesis:
//   NO geometric change.  Clone shape + material and stamp signature.
//
// Recognize:
//   Metadata-only replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace belt_replace {

constexpr const char* kSkillId = "belt_replace";

struct Input
{
    std::string belt_type = "V";       // "V" | "timing" | "flat"
    std::string belt_id   = "B-44";
    double      tension_n = 400.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace belt_replace
}  // namespace koocadcam::skill
