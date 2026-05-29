#pragma once
// @lat: [[engine/skills#food_packaging]]
//
// food_packaging — vacuum or MAP (Modified Atmosphere) food packaging.
//
//   Place product in a barrier film / tray, evacuate ambient air, and
//   either seal under vacuum or flush with a defined gas mixture
//   (CO2 / N2 / O2 blend).  Shelf-life is controlled by the residual
//   oxygen percentage.  Geometry of the product is treated as
//   unchanged; the stamp records the packaging recipe.
//
//        ┌─────────────┐                   ┌─────────────┐
//        │ open        │   food_packaging  │ sealed pack │
//        │  product    │  ──────────────▶ │  (O2 ≤ x%)  │
//        └─────────────┘                   └─────────────┘
//        geometry unchanged ; headspace gas → gas_mix at oxygen_pct
//
// Signature pattern:
//   pattern.kind             = "food_packaging"
//   pattern.package_type     = "vacuum" | "map_tray" | "map_pouch" |
//                              "flow_wrap" | "thermoform"
//   pattern.gas_mix          = "vacuum" | "n2" | "co2_n2_30_70" |
//                              "co2_n2_o2_30_65_5" | <custom>
//   pattern.oxygen_pct       = double (residual / headspace O2 percentage)
//   pattern.geometric_change = false
//
// DFM:
//   DFM-INPUT             package_type empty / unknown
//   DFM-PKG-GAS           gas_mix empty
//   DFM-PKG-OXYGEN        oxygen_pct outside [0, 21] %
//   DFM-PKG-VACUUM        package_type "vacuum" but oxygen_pct > 1 % (info)
//   DFM-PKG-HIGH-O2       oxygen_pct > 10 % (info — shelf-life risk)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace food_packaging {

constexpr const char* kSkillId = "food_packaging";

struct Input
{
    std::string  package_type  = "vacuum";   // see header
    std::string  gas_mix       = "vacuum";    // see header
    double       oxygen_pct    = 0.5;          // %
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace food_packaging
}  // namespace koocadcam::skill
