#pragma once
// @lat: [[engine/skills#wear_inspect]]
//
// wear_inspect — wear-pattern inspection of a worn surface.  The operator
// records the measured depth, classifies the pattern, and compares against
// an acceptance threshold (typically derived from drawing tolerance or a
// maintenance spec).
//
//        ┌──────────────┐                         ┌──────────────┐
//        │ worn face    │   wear_inspect          │ same face    │
//        │  ░░░░░░░░░░  │  ────────────────▶      │  ░░░░░░░░░░  │
//        │              │                         │              │
//        └──────────────┘   (depth_um, pattern,   └──────────────┘
//                            accept_threshold_um)   accept_pass /
//                                                   accept_fail
//
// Wear patterns recognised (string-typed for slice-1):
//   "uniform"   — even surface loss (normal long-term wear)
//   "galling"   — local plastic transfer / scuffing (lubrication failure)
//   "abrasive"  — scratched / grooved (foreign-particle contamination)
//
// Signature pattern:
//   - pattern.kind                = "wear_inspect"
//   - pattern.wear_depth_um       = <double>
//   - pattern.wear_pattern        = "uniform" | "galling" | "abrasive"
//   - pattern.accept_threshold_um = <double>
//   - pattern.accept_pass         = (depth ≤ threshold)
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   DFM-INPUT       wear_depth_um < 0           (error)
//   DFM-INPUT       accept_threshold_um ≤ 0     (error)
//   DFM-WEAR-PAT    pattern not in known set    (info)
//   DFM-WEAR-FAIL   depth > threshold           (info — exceedance, plan
//                   replace_part or rework)
//   DFM-WEAR-DEEP   depth > 500 μm              (info — gross wear, suspect
//                   secondary damage)
//
// Synthesis:
//   NO geometric change.  Clones the workpiece and stamps the signature.
//
// Recognize:
//   Metadata-only — surface wear of < 1 mm is below the OCCT modelling
//   resolution we treat in slice-1.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wear_inspect {

constexpr const char* kSkillId = "wear_inspect";

struct Input
{
    double       wear_depth_um       = 0.0;
    std::string  wear_pattern        = "uniform";   // "uniform"|"galling"|"abrasive"
    double       accept_threshold_um = 100.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wear_inspect
}  // namespace koocadcam::skill
