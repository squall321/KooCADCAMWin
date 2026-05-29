#pragma once
// @lat: [[engine/skills#plastic_pyrolysis]]
//
// plastic_pyrolysis — chemical recycling of mixed plastic feedstock by
// thermal decomposition (typically 400-600 °C) in an oxygen-free reactor.
// The plastic depolymerizes into a hydrocarbon condensate ("pyrolysis oil")
// plus char and syngas.  Metadata-only operation: the workpiece B-rep is
// unchanged; a yield record is stamped so downstream mass-balance and
// recovered-feedstock accounting can credit the recovered oil.
//
//   ┌────────────┐    plastic_pyrolysis    ┌──────────────────┐
//   │ feedstock  │ ─────────────────────▶ │ stamp record     │
//   │ mass_kg    │   (T °C, oil_yield%)    │ recovered oil L  │
//   └────────────┘                         └──────────────────┘
//
// Signature pattern:
//   - pattern.kind                 = "plastic_pyrolysis"
//   - pattern.feedstock_kg
//   - pattern.temp_c
//   - pattern.oil_yield_pct
//   - pattern.recovered_oil_kg     = feedstock_kg * oil_yield_pct / 100
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   feedstock_kg   > 0                            (DFM-INPUT error)
//   temp_c         ∈ [300, 800]                   (DFM-PYRO-TEMP info if outside)
//   oil_yield_pct  ∈ (0, 100]                     (DFM-INPUT error)
//   oil_yield_pct  ≤ 85 typical ceiling           (DFM-PYRO-YIELD info if exceeded)
//
// Synthesis: NO geometric change.  Clone + stamp signature.
// Recognize: Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace plastic_pyrolysis {

constexpr const char* kSkillId = "plastic_pyrolysis";

struct Input
{
    double  feedstock_kg   = 100.0;     // mixed plastic feed mass
    double  temp_c         = 500.0;     // reactor temperature
    double  oil_yield_pct  = 65.0;      // pyrolysis-oil mass fraction
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace plastic_pyrolysis
}  // namespace koocadcam::skill
