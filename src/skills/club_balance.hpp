#pragma once
// @lat: [[engine/skills#club_balance]]
//
// club_balance — metadata-only record that a golf club has been balanced
// (swing-weight measured + tip-weight tuned) for a given head mass.  The
// B-rep of the club shaft/head is unchanged; this skill stamps the swing
// weight code and the head mass used during the measurement.
//
// Signature pattern:
//   - pattern.kind             = "club_balance"
//   - pattern.swing_weight     = "C9" .. "E5" (alpha-numeric scale: A0..F9)
//   - pattern.head_mass_g      = double (grams, typical 195-220 g)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   swing_weight matches /^[A-F][0-9]$/                (DFM-CLUB-SW error)
//   head_mass_g ∈ [150, 350] g                         (DFM-CLUB-MASS error)
//   head_mass_g ∈ [180, 230] g (driver typical)        (DFM-CLUB-MASS info if outside)
//
// Synthesis: NO geometric change.  Recognize: metadata-only replay.

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace club_balance {

constexpr const char* kSkillId = "club_balance";

struct Input
{
    std::string swing_weight = "D2";    // "A0".."F9"
    double      head_mass_g  = 200.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace club_balance
}  // namespace koocadcam::skill
