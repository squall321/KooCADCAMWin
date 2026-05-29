#pragma once
// @lat: [[engine/skills#racket_string]]
//
// racket_string — metadata-only record that a sports racket frame has been
// strung at a target tension following a string pattern (e.g. "16x19").
// The B-rep of the frame is unchanged; this skill stamps the stringing
// parameters so QA can downstream verify the build.
//
// Signature pattern:
//   - pattern.kind             = "racket_string"
//   - pattern.tension_lbf      = double (pounds-force, typical 40-70 lbf)
//   - pattern.string_pattern   = "16x19" / "18x20" / …
//   - pattern.main_count       = parsed int from pattern (16, 18, …)
//   - pattern.cross_count      = parsed int from pattern (19, 20, …)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   tension_lbf  ∈ [30, 80]           (DFM-RACKET-TENSION error if outside)
//   string_pattern matches "<m>x<c>"  (DFM-RACKET-PATTERN error if malformed)
//   main_count   ∈ [12, 22]           (DFM-RACKET-PATTERN info if unusual)
//
// Synthesis: NO geometric change.  Recognize: metadata-only replay.

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace racket_string {

constexpr const char* kSkillId = "racket_string";

struct Input
{
    double      tension_lbf   = 55.0;
    std::string string_pattern = "16x19";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace racket_string
}  // namespace koocadcam::skill
