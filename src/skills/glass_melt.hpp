#pragma once
// @lat: [[engine/skills#glass_melt]]
//
// glass_melt — fuse a raw glass batch (silica + soda + lime + cullet) into
// a homogeneous molten pool.
//
// A measured batch composition is charged into a refractory furnace,
// ramped past the melt temperature to dissolve all sand grains, then held
// at the higher *fining* temperature to drive off seeds (gas bubbles) and
// homogenize the melt before forming.  Because the workpiece at this stage
// is a bulk liquid, the modeling layer treats apply() as a *no-op on
// geometry* and records the batch + temperatures as metadata.  Downstream
// forming skills (float_glass, glass_blow_mold) will set the actual shape.
//
//        ┌─────────────┐    melt + fine     ┌──────────────────┐
//        │ raw batch   │  ───────────────▶ │ homogeneous melt │
//        │ (SiO2,Na2O, │  (T_melt, T_fine) │ (no bubbles)     │
//        │  CaO, cullet│                    │                  │
//        └─────────────┘                    └──────────────────┘
//
// Signature pattern:
//   pattern.kind             == "glass_melt"
//   pattern.melt_temp_c, pattern.fining_temp_c
//   pattern.batch_composition (free-form JSON object)
//   pattern.geometric_change == false
//
// DFM checks (errors block apply):
//   DFM-MELT-TEMP    : melt_temp_c ∈ [1100, 1700] °C
//   DFM-MELT-FINING  : fining_temp_c ∈ [melt_temp_c, 1700] °C  (must be ≥)
//   DFM-MELT-BATCH   : batch_composition non-empty
//
// Recognize: metadata-only.  Melt history cannot be inferred from a B-Rep.

#include "Datum.hpp"
#include "Skill.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace glass_melt {

constexpr const char* kSkillId = "glass_melt";

struct Input
{
    double           melt_temp_c       = 1450.0;
    double           fining_temp_c     = 1550.0;
    nlohmann::json   batch_composition = {
        { "SiO2",  72.0 },
        { "Na2O",  14.0 },
        { "CaO",    9.0 },
        { "MgO",    4.0 },
        { "Al2O3",  1.0 },
    };
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace glass_melt
}  // namespace koocadcam::skill
