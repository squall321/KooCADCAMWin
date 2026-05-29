#pragma once
// @lat: [[engine/skills#payload_qualify]]
//
// payload_qualify — metadata-only record of a robot payload qualification
// run: the robot lifted `tested_mass_kg`, the safety brakes were exercised,
// and the maximum continuous speed (% of rated) was demonstrated.  No
// geometric change to the workpiece.
//
// Signature pattern:
//   - pattern.kind               = "payload_qualify"
//   - pattern.tested_mass_kg     = <input>
//   - pattern.max_speed_pct      = <input>
//   - pattern.brake_test_passed  = <input>
//   - pattern.geometry_changed   = false
//
// DFM checks:
//   tested_mass_kg > 0                      (DFM-INPUT error)
//   max_speed_pct ∈ [0, 100]                (DFM-PAYLOAD-SPEED error if outside)
//   brake_test_passed == false              (DFM-PAYLOAD-BRAKE error — brakes failed)
//   max_speed_pct < 25                      (DFM-PAYLOAD-SPEED info — derated)
//
// Synthesis:
//   NO geometric change.  Clone shape+material verbatim, copy prior
//   features, append qualification signature.
//
// Recognize:
//   Metadata-only — replay signatures from workpiece.features().

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace payload_qualify {

constexpr const char* kSkillId = "payload_qualify";

struct Input
{
    double tested_mass_kg     = 10.0;
    double max_speed_pct      = 100.0;
    bool   brake_test_passed  = true;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace payload_qualify
}  // namespace koocadcam::skill
