#pragma once
// @lat: [[engine/skills#photomask_pattern]]
//
// photomask_pattern — lithography step: a reticle pattern is projected onto
// the photoresist-coated wafer; exposure transfers the mask features to the
// resist for the subsequent develop + etch.  GEOMETRY is unchanged at the
// macroscopic OCCT level (resist exposure is sub-micron); only the
// signature records the litho parameters.
//
//        ┌──────────┐   ╔══════╗            ┌────────────┐
//        │  wafer + │ + ║ mask ║  exposure  │ wafer +    │
//        │  resist  │   ╚══════╝  ────────▶ │ latent img │
//        └──────────┘                        └────────────┘
//
// Signature pattern:
//   - pattern.kind                 = "photomask_pattern"
//   - pattern.mask_id              = <input>
//   - pattern.exposure_dose_mj_cm2 = <input>
//   - pattern.alignment_class      = <input>   "coarse" | "fine" | "critical"
//   - pattern.is_metadata_only     = true
//
// DFM checks:
//   mask_id non-empty
//   exposure_dose_mj_cm2 > 0  (1 – 1000 mJ/cm² typical;  outside is info)
//   alignment_class ∈ {coarse, fine, critical}
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace photomask_pattern {

constexpr const char* kSkillId = "photomask_pattern";

struct Input
{
    std::string  mask_id              = "M-DEFAULT-01";
    double       exposure_dose_mj_cm2 = 30.0;
    std::string  alignment_class      = "fine";   // coarse | fine | critical
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace photomask_pattern
}  // namespace koocadcam::skill
