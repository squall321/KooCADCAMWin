#pragma once
// @lat: [[engine/skills#primer_install]]
//
// primer_install — record seating of a primer cup into the case primer
// pocket for authorized civilian/commercial small-arms manufacturing.  The
// case head B-rep already has the primer pocket from upstream operations;
// this skill stamps the installed-primer metadata so process planning can
// downstream verify seating depth + brand traceability.
//
// Signature pattern:
//   - pattern.kind             = "primer_install"
//   - pattern.primer_size      = "small_pistol" | "large_pistol" |
//                                "small_rifle"  | "large_rifle"  | "magnum"
//   - pattern.primer_brand     = supplier name (traceability)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   primer_size   ∈ {small_pistol, large_pistol, small_rifle,
//                    large_rifle, magnum}        (DFM-AMMO-PRIMER info if unknown)
//   primer_brand  non-empty                       (DFM-AMMO-BRAND error)
//
// Synthesis: metadata-only.
// Recognize: metadata replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace primer_install {

constexpr const char* kSkillId = "primer_install";

struct Input
{
    std::string primer_size  = "small_pistol";
    std::string primer_brand = "CCI";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace primer_install
}  // namespace koocadcam::skill
