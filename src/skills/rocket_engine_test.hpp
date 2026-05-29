#pragma once
// @lat: [[engine/skills#rocket_engine_test]]
//
// rocket_engine_test — hot-fire qualification record for a liquid- or
// solid-propellant rocket engine.  Captures engine identifier, measured
// thrust (kN), and burn duration (s).  The engine hardware B-rep is
// unchanged by the test itself — this skill stamps the qualification
// metadata so the propulsion certification chain can downstream verify
// the firing campaign.
//
// Signature pattern:
//   - pattern.kind             = "rocket_engine_test"
//   - pattern.engine_id        = engine serial/build identifier
//   - pattern.thrust_kn        = measured thrust (kN)
//   - pattern.burn_time_s      = firing duration (s)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   engine_id     non-empty               (DFM-INPUT error)
//   thrust_kn     ∈ (0, 50000]            (DFM-SPACE-THRUST error if outside)
//   burn_time_s   ∈ (0, 1800]             (DFM-SPACE-BURN error if outside)
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

namespace rocket_engine_test {

constexpr const char* kSkillId = "rocket_engine_test";

struct Input
{
    std::string engine_id    = "ENG-0001";
    double      thrust_kn    = 100.0;
    double      burn_time_s  = 120.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace rocket_engine_test
}  // namespace koocadcam::skill
