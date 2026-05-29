#pragma once
// @lat: [[engine/skills#solar_array_deploy_test]]
//
// solar_array_deploy_test — gravity-offload deployment qualification of a
// folded photovoltaic array.  Captures the array identifier, number of
// deploy/restow cycles, and measured release-latch torque (N·m).  The
// array B-rep is unchanged — the test is a non-destructive functional
// qualification, so this skill stamps the campaign metadata.
//
// Signature pattern:
//   - pattern.kind              = "solar_array_deploy_test"
//   - pattern.array_id          = array serial identifier
//   - pattern.cycles_count      = number of deploy/restow cycles
//   - pattern.latch_torque_nm   = release-latch break-out torque (N·m)
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   array_id          non-empty           (DFM-INPUT error)
//   cycles_count      ∈ [1, 100]          (DFM-SPACE-DEPLOY error if outside)
//   latch_torque_nm   ∈ (0, 500]          (DFM-SPACE-LATCH error if outside)
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

namespace solar_array_deploy_test {

constexpr const char* kSkillId = "solar_array_deploy_test";

struct Input
{
    std::string array_id        = "SA-0001";
    int         cycles_count    = 5;
    double      latch_torque_nm = 12.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace solar_array_deploy_test
}  // namespace koocadcam::skill
