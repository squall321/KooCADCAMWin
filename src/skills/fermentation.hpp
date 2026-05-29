#pragma once
// @lat: [[engine/skills#fermentation]]
//
// fermentation — microbial fermentation of food substrate.
//
//   Inoculate a substrate with a starter culture, hold under controlled
//   temperature for the required time, and let metabolism drive the
//   pH down (or alcohol up) until the target pH is reached.  Geometry
//   is treated as unchanged; the stamp records the recipe so QA can
//   verify pH endpoint and starter identity for traceability.
//
//        ┌─────────────┐                ┌─────────────┐
//        │ substrate   │  fermentation  │ fermented   │
//        │  + starter  │  ──────────▶  │  (pH ≤ tgt) │
//        └─────────────┘                └─────────────┘
//        geometry unchanged ; pH → ph_target
//
// Signature pattern:
//   pattern.kind             = "fermentation"
//   pattern.temp_c           = double (incubation °C)
//   pattern.time_h           = double (hours)
//   pattern.starter_culture  = string (strain / blend identifier)
//   pattern.ph_target        = double (endpoint pH)
//   pattern.geometric_change = false
//
// DFM:
//   DFM-INPUT            starter_culture empty
//   DFM-FERM-TEMP        temp_c outside (0, 60] °C
//   DFM-FERM-TIME        time_h outside (0, 720] hours
//   DFM-FERM-PH          ph_target outside [2.5, 7.0]
//   DFM-FERM-PH-HIGH     ph_target > 5.5 (info — insufficient acidification
//                        for many fermented food safety models)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace fermentation {

constexpr const char* kSkillId = "fermentation";

struct Input
{
    double       temp_c          = 30.0;                  // °C
    double       time_h          = 24.0;                  // hours
    std::string  starter_culture = "lactobacillus_plantarum";
    double       ph_target       = 4.5;                   // endpoint pH
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace fermentation
}  // namespace koocadcam::skill
