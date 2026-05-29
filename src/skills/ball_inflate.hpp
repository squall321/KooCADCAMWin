#pragma once
// @lat: [[engine/skills#ball_inflate]]
//
// ball_inflate — metadata-only record that an inflatable ball has been
// brought to a target gauge pressure.  The B-rep of the ball shell is
// unchanged (the shell radius does not meaningfully expand under playing
// pressure); this skill stamps the target pressure and ball type so QA can
// later verify against league-spec ranges.
//
// Signature pattern:
//   - pattern.kind                = "ball_inflate"
//   - pattern.target_pressure_psi = double
//   - pattern.ball_type           = "soccer" | "basketball" | "football" | …
//   - pattern.in_spec_range       = bool (target within league spec)
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   target_pressure_psi > 0                        (DFM-INPUT error)
//   target_pressure_psi <= 30 psi                  (DFM-BALL-PRESSURE error if > 30)
//   target_pressure_psi within type-specific spec  (DFM-BALL-PRESSURE info)
//   ball_type non-empty                            (DFM-INPUT warning)
//
// Type-specific spec ranges (gauge psi):
//   soccer      → [8.5, 15.6]
//   basketball  → [7.5, 8.5]
//   football    → [12.5, 13.5]
//   volleyball  → [4.3, 4.6]
//   rugby       → [9.5, 10.0]
//
// Synthesis: NO geometric change.  Recognize: metadata-only replay.

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ball_inflate {

constexpr const char* kSkillId = "ball_inflate";

struct Input
{
    double      target_pressure_psi = 9.0;
    std::string ball_type           = "soccer";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ball_inflate
}  // namespace koocadcam::skill
