#pragma once
// @lat: [[engine/skills#vacuum_pack]]
//
// vacuum_pack — evacuate ambient air from a flexible bag and seal under
// vacuum.  Used for shelf-life extension (foodstuffs), corrosion control
// (steel parts), and volume reduction (textile shipping).  The vacuum is
// expressed in millibar absolute pressure: lower number → harder vacuum.
//
//        ┌─────────────┐                  ┌─────────────┐
//        │  loaded bag │   vacuum_pack    │  evacuated  │
//        │  (1013 mbar)│ ───────────────▶│   + sealed   │
//        │             │ (vac, seal time) │  (X mbar)    │
//        └─────────────┘                  └─────────────┘
//        geometry unchanged ; vacuum + seal metadata stamped
//
// Signature pattern:
//   pattern.kind             = "vacuum_pack"
//   pattern.vacuum_mbar      = double  (residual absolute pressure)
//   pattern.seal_time_s      = double
//   pattern.geometry_changed = false
//
// DFM:
//   DFM-INPUT             vacuum_mbar ≤ 0 or seal_time_s ≤ 0
//   DFM-VAC-WEAK          vacuum_mbar > 100 (info — weak vacuum)
//   DFM-VAC-TIME-LONG     seal_time_s > 10 (info — long cycle / energy cost)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace vacuum_pack {

constexpr const char* kSkillId = "vacuum_pack";

struct Input
{
    double  vacuum_mbar  = 5.0;
    double  seal_time_s  = 2.5;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace vacuum_pack
}  // namespace koocadcam::skill
