#pragma once
// @lat: [[engine/skills#pickling_passivate]]
//
// pickling_passivate — surface chemical conditioning for stainless / nickel
// alloy hull plate.  An acid mix (typically HNO3 + HF) strips mill scale,
// weld heat-tint, and free iron, leaving a clean chromium-rich surface that
// then PASSIVATES (re-forms the protective Cr2O3 oxide) in dilute HNO3.
// Macroscopic geometry is unchanged at the planner level; only the surface
// chemistry changes.
//
//        ┌──────────────────────┐    pickle (acid bath)
//        │  scale + weld tint   │  ─────────────────▶  bare clean steel
//        └──────────────────────┘                              │
//                                                     passivate (HNO3)
//                                                              ▼
//                                                  Cr2O3 protective film
//
// Signature pattern:
//   - pattern.kind                = "pickling_passivate"
//   - pattern.surface_area_m2     = <input>
//   - pattern.acid_mix            = "HNO3+HF" | "HNO3" | "citric" | ...
//   - pattern.passivation_time_h  = <input>
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   surface_area_m2     > 0                              (DFM-INPUT error)
//   acid_mix            non-empty                        (DFM-INPUT warning)
//   acid_mix            ∈ {HNO3+HF, HNO3, citric}        (DFM-PICK-MIX info)
//   passivation_time_h  ∈ [0.5, 4.0] (typical ASTM A967) (DFM-PICK-TIME info)
//   passivation_time_h  > 0                              (DFM-INPUT error)
//
// Synthesis: clone shape + material, stamp signature.
// Recognize: metadata-only (surface chemistry is sub-OCCT).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pickling_passivate {

constexpr const char* kSkillId = "pickling_passivate";

struct Input
{
    double      surface_area_m2    = 20.0;
    std::string acid_mix           = "HNO3+HF";   // ASTM A380 type
    double      passivation_time_h = 1.5;          // typical 0.5–4.0 h
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pickling_passivate
}  // namespace koocadcam::skill
