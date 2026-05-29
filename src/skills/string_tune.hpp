#pragma once
// @lat: [[engine/skills#string_tune]]
//
// string_tune — metadata-only record that the strings of a stringed
// instrument have been brought to target pitches.  The B-rep is unchanged;
// the skill stamps each string's target frequency in Hz.
//
// Signature pattern:
//   - pattern.kind                    = "string_tune"
//   - pattern.string_count            = int (>= 1)
//   - pattern.target_freq_hz_per_string = array of doubles (one per string)
//   - pattern.geometry_changed        = false
//
// DFM checks:
//   string_count >= 1                                        (DFM-INPUT error)
//   target_freq_hz_per_string.size() == string_count         (DFM-TUNE-COUNT error)
//   all frequencies in [16, 8000] Hz (musical range)         (DFM-TUNE-FREQ error)
//
// Synthesis: NO geometric change.  Recognize: metadata-only replay.

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace string_tune {

constexpr const char* kSkillId = "string_tune";

struct Input
{
    int                 string_count = 6;
    std::vector<double> target_freq_hz_per_string =
        { 82.41, 110.00, 146.83, 196.00, 246.94, 329.63 };  // E2 A2 D3 G3 B3 E4
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace string_tune
}  // namespace koocadcam::skill
