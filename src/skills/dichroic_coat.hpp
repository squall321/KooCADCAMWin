#pragma once
// @lat: [[engine/skills#dichroic_coat]]
//
// dichroic_coat — interference-filter multilayer that *reflects* one
// portion of the spectrum and *transmits* the complement at a sharp
// cutoff wavelength.  Used for beam-splitting, fluorescence
// excitation/emission separation, cold mirrors in projectors, etc.
//
//        T ▲              transmit band
//          │             ┌──────────────
//      1.0 │             │
//          │             │ ← sharp cutoff at λ_c
//          │  reflect    │
//      0.0 │  band ─────┘
//          └───────────────────────▶  λ
//
// Signature pattern:
//   - pattern.kind                   = "dichroic_coat"
//   - pattern.target_face_id         = face that was coated
//   - pattern.cutoff_wavelength_nm   = transition wavelength
//   - pattern.transmission_band_nm   = [λ_min, λ_max] passed
//   - pattern.reflection_band_nm     = [λ_min, λ_max] reflected
//   - pattern.geometry_changed       = false
//
// DFM checks:
//   cutoff_wavelength_nm in [200, 14000]                          (error if out)
//   transmission / reflection bands non-degenerate (λ_min < λ_max) (error)
//   cutoff sits inside one band's span                             (info hint)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace dichroic_coat {

constexpr const char* kSkillId = "dichroic_coat";

struct Input
{
    FaceDatum entry_face;
    double    cutoff_wavelength_nm     = 500.0;
    double    transmission_min_nm      = 500.0;
    double    transmission_max_nm      = 700.0;
    double    reflection_min_nm        = 380.0;
    double    reflection_max_nm        = 500.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace dichroic_coat
}  // namespace koocadcam::skill
