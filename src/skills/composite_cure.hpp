#pragma once
// @lat: [[engine/skills#composite_cure]]
//
// composite_cure — autoclave or oven cure of a laid-up composite stack.
//
// After a `composite_layup`, the part is bagged and either placed in an
// autoclave (heat + pressure + vacuum) or an oven (heat + vacuum only).
// The cure crosslinks the matrix resin into its final glass / rubbery state.
// Macroscopic geometry is treated as UNCHANGED at the modeling layer (in
// reality there is small spring-back / chemical shrink; we abstract that
// away — the workpiece envelope IS the cured envelope).
//
//        autoclave:   T = temp_c,  p = pressure_kpa,  vacuum = on
//        oven:        T = temp_c,  p = 0,              vacuum = on/off
//
// Signature pattern:
//   pattern.kind            == "composite_cure"
//   pattern.temp_c, pattern.time_min, pattern.pressure_kpa, pattern.vacuum
//   pattern.cure_method      ("autoclave" | "oven")
//   pattern.geometric_change == false
//
// DFM checks:
//   DFM-CURE-TEMP   : temp_c ∈ [80, 220] °C  (epoxy operating window)
//   DFM-CURE-TIME   : time_min ≥ 5 (else under-cure)
//   DFM-CURE-PRESS  : pressure_kpa ∈ [0, 1500] kPa
//   DFM-CURE-METHOD : info — autoclave inferred when pressure_kpa > 100 kPa
//
// Recognize: metadata-only (cure leaves no B-Rep trace).

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace composite_cure {

constexpr const char* kSkillId = "composite_cure";

struct Input
{
    double  temp_c          = 120.0;
    double  time_min        = 120.0;
    double  pressure_kpa    = 600.0;   // > 100 → autoclave; ≤ 100 → oven
    bool    vacuum          = true;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace composite_cure
}  // namespace koocadcam::skill
