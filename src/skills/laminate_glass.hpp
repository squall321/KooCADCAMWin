#pragma once
// @lat: [[engine/skills#laminate_glass]]
//
// laminate_glass — bond two glass plies together with a polymer interlayer
// (typically polyvinyl butyral, PVB) under heat and pressure.
//
// Two cut + edged glass sheets sandwich a PVB film of nominal
// `interlayer_thickness_mm` (commonly 0.38 / 0.76 / 1.52 mm).  The stack is
// nipped to expel air, then cycled in an autoclave at `autoclave_temp_c`
// (typically ~ 140 °C) and `autoclave_pressure_kpa` (~ 1200 kPa) so that
// the PVB clears and bonds chemically to the glass.  Macroscopic geometry
// of the laminate IS the outer envelope of the stack; modeling treats it
// as a no-op and records process parameters.
//
//        glass  ┌─────────────┐         autoclave         ┌──────────────────┐
//        glass  │ glass / PVB │  ───────────────────▶   │ bonded laminate  │
//        PVB    │ / glass     │  (T, p, t)                │ optically clear  │
//               └─────────────┘                            └──────────────────┘
//
// Signature pattern:
//   pattern.kind                  == "laminate_glass"
//   pattern.interlayer_thickness_mm, pattern.autoclave_temp_c,
//   pattern.autoclave_pressure_kpa
//   pattern.geometric_change      == false
//
// DFM checks (errors block apply):
//   DFM-LAMI-INTERLAYER : interlayer_thickness_mm ∈ [0.38, 2.28]
//   DFM-LAMI-TEMP       : autoclave_temp_c ∈ [120, 160] °C
//   DFM-LAMI-PRESSURE   : autoclave_pressure_kpa ∈ [800, 1500] kPa
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace laminate_glass {

constexpr const char* kSkillId = "laminate_glass";

struct Input
{
    double  interlayer_thickness_mm  = 0.76;
    double  autoclave_temp_c         = 140.0;
    double  autoclave_pressure_kpa   = 1200.0;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace laminate_glass
}  // namespace koocadcam::skill
