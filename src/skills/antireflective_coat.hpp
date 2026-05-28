#pragma once
// @lat: [[engine/skills#antireflective_coat]]
//
// antireflective_coat — vacuum-deposited multilayer dielectric stack
// (MgF₂ / SiO₂ / TiO₂ / Ta₂O₅ alternating high-/low-index layers) that
// drives the reflectance of an optical face below ~0.5 % across a
// specified wavelength band.  Geometric thickness of the entire stack is
// typically < 1 μm — invisible at OCCT scale — so the stack is recorded
// only as signature metadata.
//
//        ┌─────────────┐
//        │ ░░░░░░░░░░░ │   ← AR multilayer stack (few hundred nm total)
//        │             │
//        │   glass     │   ← optical substrate (geometry unchanged)
//        │   substrate │
//        └─────────────┘
//
// Signature pattern:
//   - pattern.kind                  = "antireflective_coat"
//   - pattern.target_face_id        = face that was coated
//   - pattern.wavelength_band_nm    = [λ_min, λ_max] design band
//   - pattern.reflectance_pct       = target average R% in band
//   - pattern.layer_count           = number of deposited layers
//   - pattern.geometry_changed      = false
//
// DFM checks:
//   wavelength_band_nm: λ_min < λ_max, both > 0                   (error)
//   wavelength_band_nm: within 200 – 14000 nm                     (info)
//   reflectance_pct in (0, 10]                                    (error if out)
//   layer_count ≥ 1                                               (error)
//   layer_count > 30                                              (info — exotic stack)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace antireflective_coat {

constexpr const char* kSkillId = "antireflective_coat";

struct Input
{
    FaceDatum entry_face;
    double    wavelength_min_nm  = 450.0;
    double    wavelength_max_nm  = 650.0;
    double    reflectance_pct    = 0.5;
    int       layer_count        = 7;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace antireflective_coat
}  // namespace koocadcam::skill
