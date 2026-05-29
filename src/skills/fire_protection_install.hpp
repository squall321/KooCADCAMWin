#pragma once
// @lat: [[engine/skills#fire_protection_install]]
//
// fire_protection_install — installation record for a wet-pipe sprinkler
// fire-suppression system.  Sprinkler heads + supply piping are sub-OCCT
// MEP entities so the host workpiece geometry is unchanged; the system
// is captured as metadata for NFPA-13 coverage / hydraulic checks.
//
// Signature pattern:
//   - pattern.kind                = "fire_protection_install"
//   - pattern.sprinkler_count     = number of installed heads
//   - pattern.supply_pipe_dia_mm  = main feed pipe Ø
//   - pattern.coverage_m2         = total protected floor area
//   - pattern.coverage_per_head_m2 = derived
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   sprinkler_count     ≥ 1                  (DFM-INPUT error)
//   supply_pipe_dia_mm  ∈ (0, 300]           (DFM-FIRE-DIA error)
//   coverage_m2         > 0                  (DFM-INPUT error)
//   coverage / head     > 20 m^2             (DFM-FIRE-COVERAGE info, light-hazard NFPA-13 limit)

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace fire_protection_install {

constexpr const char* kSkillId = "fire_protection_install";

struct Input
{
    int    sprinkler_count    = 12;
    double supply_pipe_dia_mm = 65.0;
    double coverage_m2        = 200.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fire_protection_install
}  // namespace koocadcam::skill
