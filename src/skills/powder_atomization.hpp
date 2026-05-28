#pragma once
// @lat: [[engine/skills#powder_atomization]]
//
// powder_atomization — produce a fine metal powder from a molten stream by
// breaking it into droplets that solidify mid-flight.  Three industrial
// variants:
//
//   gas atomization      inert-gas jet (Ar/N2)   spherical, 5-100 µm,
//                                                MIM/AM grade, premium
//   water atomization    high-pressure water jet irregular, 30-200 µm,
//                                                press-and-sinter grade
//   plasma atomization   plasma torch on wire    very spherical, AM grade
//
//      ┌──────────┐  atomize   ┌──────────────┐  classify   ┌────────────┐
//      │  molten  │  ────────▶ │  droplets +  │   ───────▶  │  powder    │
//      │  stream  │  jet       │  loose powder│   sieve     │  d50, yield│
//      └──────────┘            └──────────────┘             └────────────┘
//
// Model: the workpiece is a STAND-IN bulk-container representing one batch
// of powder.  Atomization does not modify a B-rep — it produces a powder
// LOT.  We record the powder spec on the feature history so downstream
// powder_blend / metal_injection_mold can reference it.
//
// DFM:
//   DFM-PA-D50          : median_particle_size_um in (0, 500]; info if outside
//                         [5, 200] (out-of-spec for MIM / press-and-sinter).
//   DFM-PA-GAS          : gas in {argon, nitrogen, water, plasma, air}.
//   DFM-PA-YIELD        : yield_pct in (0, 100].  Info if < 50 (process upset).
//   DFM-PA-MAT          : material in atomizable catalog (info on unknowns).
//
// Signature pattern:
//   pattern.kind                       == "powder_atomization"
//   pattern.median_particle_size_um, pattern.gas, pattern.yield_pct
//   pattern.process_family             == "powder_metallurgy"
//   pattern.geometric_change           == false
//
// Recognize:
//   Metadata-only — replays history.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace powder_atomization {

constexpr const char* kSkillId = "powder_atomization";

struct Input
{
    std::string  material                  = "stainless_316l";
    double       median_particle_size_um   = 25.0;     // d50
    std::string  gas                       = "argon";  // argon|nitrogen|water|plasma|air
    double       yield_pct                 = 75.0;     // of starting melt mass
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace powder_atomization
}  // namespace koocadcam::skill
