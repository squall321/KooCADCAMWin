#pragma once
// @lat: [[engine/skills#sterilize]]
//
// sterilize — terminal sterilization of a medical-grade part.
//
//   method:
//     "autoclave" → moist heat (121 °C / 134 °C, ~ 15–30 min)
//     "eto"       → ethylene oxide gas (~ 600 mg/L, ~ 1–6 h)
//     "gamma"     → gamma irradiation (~ 25–50 kGy)
//     "ebeam"     → electron beam (~ 15–40 kGy)
//     "h2o2_plasma" → vaporized hydrogen peroxide plasma
//
//        ┌─────────────┐                ┌─────────────┐
//        │ part        │   sterilize    │ part        │
//        │  (bioburden)│  ──────────▶  │  (SAL 10⁻⁶) │
//        └─────────────┘                └─────────────┘
//        geometry unchanged ; surface bioburden → ≤ SAL 10⁻⁶
//
// Signature pattern:
//   pattern.kind            = "sterilize"
//   pattern.method          = <input>
//   pattern.dose_or_temp    = double (kGy for gamma/ebeam, °C for autoclave,
//                                     mg/L for EtO concentration)
//   pattern.exposure_min    = double  (cycle exposure minutes)
//   pattern.geometric_change= false
//
// DFM:
//   DFM-INPUT             method empty / unknown
//   DFM-STERILIZE-EXPO    exposure_min ∈ (0, 1440] (1 minute … 24 hours)
//   DFM-STERILIZE-DOSE    dose_or_temp ≤ 0 OR outside reasonable per-method band
//
// Recognize: metadata-only.  Sterilization leaves no analytic-B-Rep trace.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sterilize {

constexpr const char* kSkillId = "sterilize";

struct Input
{
    std::string  method        = "autoclave";   // see header
    double       dose_or_temp  = 121.0;          // °C or kGy or mg/L per method
    double       exposure_min  = 30.0;           // minutes
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace sterilize
}  // namespace koocadcam::skill
