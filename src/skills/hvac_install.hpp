#pragma once
// @lat: [[engine/skills#hvac_install]]
//
// hvac_install — installation record for an HVAC duct + register + fan
// assembly.  The duct, registers, and fan are sub-OCCT MEP entities so the
// host workpiece geometry is preserved; this skill stamps the installed
// HVAC metadata for downstream load / acoustic / commissioning checks.
//
// Signature pattern:
//   - pattern.kind            = "hvac_install"
//   - pattern.duct_size_mm    = nominal duct hydraulic Ø
//   - pattern.total_length_m  = total duct run length
//   - pattern.register_count  = supply/return register count
//   - pattern.fan_kw          = fan motor rated power
//   - pattern.geometry_changed = false
//
// DFM checks:
//   duct_size_mm    ∈ (0, 2000]              (DFM-HVAC-DUCT error if outside)
//   total_length_m  > 0                      (DFM-INPUT error)
//   register_count  ≥ 1                      (DFM-INPUT error if < 1)
//   fan_kw          ∈ (0, 500]               (DFM-HVAC-FAN error)
//   total_length_m / register_count > 30 m   (DFM-HVAC-COVERAGE info, long throw)

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hvac_install {

constexpr const char* kSkillId = "hvac_install";

struct Input
{
    double duct_size_mm   = 300.0;
    double total_length_m = 40.0;
    int    register_count = 6;
    double fan_kw         = 5.5;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hvac_install
}  // namespace koocadcam::skill
