#pragma once
// @lat: [[engine/skills#mirror_finish]]
//
// mirror_finish — high-reflectivity metallic or dielectric coating on
// an optical face (Al, Ag, Au, enhanced Al, protected Ag, broadband
// dielectric).  Typically a thin metal layer (< 200 nm) plus a
// protective dielectric overcoat; geometric change is negligible.
//
//        ┌─────────────┐
//        │ ▓▓▓▓▓▓▓▓▓▓▓ │   ← reflective metal layer + protective overcoat
//        │             │
//        │   polished  │   ← polished optical substrate
//        │   substrate │
//        └─────────────┘
//
// Signature pattern:
//   - pattern.kind                   = "mirror_finish"
//   - pattern.target_face_id         = face that was coated
//   - pattern.reflectivity_pct       = average R% across wavelength_band
//   - pattern.wavelength_band_nm     = [λ_min, λ_max]
//   - pattern.geometry_changed       = false
//
// DFM checks:
//   reflectivity_pct in (0, 100]                                  (error if out)
//   reflectivity_pct < 70 → not a "mirror"                        (info)
//   reflectivity_pct ≥ 99.5 → enhanced/protected/dielectric class (info)
//   wavelength bounds positive + ordered                          (error)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mirror_finish {

constexpr const char* kSkillId = "mirror_finish";

struct Input
{
    FaceDatum entry_face;
    double    reflectivity_pct    = 95.0;
    double    wavelength_min_nm   = 400.0;
    double    wavelength_max_nm   = 700.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mirror_finish
}  // namespace koocadcam::skill
