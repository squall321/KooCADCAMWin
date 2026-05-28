#pragma once
// @lat: [[engine/skills#resin_transfer_mold]]
//
// resin_transfer_mold (RTM) — inject thermoset resin into a closed mold
// containing a dry fiber preform.  Geometry is NOT modified at the modeling
// layer (the workpiece IS the molded shape).  The skill records process
// parameters: injection pressure, gate count, vent count.
//
//     ╔═════════════════════════════════╗
//     ║     ━━━━━━ vent ↑ ━━━━━━        ║
//     ║   (closed mold + dry preform)   ║
//     ║     ━━━━━━ gate ↑ ━━━━━━        ║   gates feed resin (pressure)
//     ╚═════════════════════════════════╝
//
// Signature pattern:
//   pattern.kind                    == "resin_transfer_mold"
//   pattern.injection_pressure_kpa, pattern.gate_count, pattern.vent_count
//   pattern.geometric_change        == false
//
// DFM checks:
//   DFM-RTM-PRESS  : injection_pressure_kpa ∈ [100, 3000] kPa
//   DFM-RTM-GATES  : gate_count ≥ 1
//   DFM-RTM-VENTS  : vent_count ≥ 1
//   DFM-RTM-RATIO  : info — gates/vents ratio > 5:1 risks dry spots
//
// Recognize: metadata-only (gates / vents are mold-tool features, not part
// features — they don't appear on the cured workpiece).

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace resin_transfer_mold {

constexpr const char* kSkillId = "resin_transfer_mold";

struct Input
{
    double  injection_pressure_kpa = 700.0;
    int     gate_count             = 1;
    int     vent_count             = 1;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace resin_transfer_mold
}  // namespace koocadcam::skill
