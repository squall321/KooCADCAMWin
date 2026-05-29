#pragma once
// @lat: [[engine/skills#temper_glass]]
//
// temper_glass — thermal tempering of flat (or shaped) glass.
//
// The glass sheet is heated past its strain point — typically `peak_temp_c`
// in the 600–680 °C range — then rapidly quenched with high-velocity air
// jets at `quench_temp_c` (≈ ambient).  The surfaces cool first and lock in
// compressive residual stress; the interior solidifies later in tension.
// The resulting `surface_stress_mpa` (target ≥ 69 MPa for safety glass)
// gives the part ~ 4× the strength of annealed glass and a dice-like
// fracture pattern.  Macroscopic geometry is unchanged; the change is
// purely in the residual-stress field.
//
//        annealed glass  ─▶ heat (600-680 °C) ─▶ air-quench ─▶ tempered glass
//                                                              σ_surf ≈ -100 MPa
//
// Signature pattern:
//   pattern.kind                == "temper_glass"
//   pattern.peak_temp_c, pattern.quench_temp_c, pattern.surface_stress_mpa
//   pattern.geometric_change    == false
//
// DFM checks (errors block apply):
//   DFM-TEMPER-PEAK    : peak_temp_c ∈ [600, 700] °C
//   DFM-TEMPER-QUENCH  : quench_temp_c ∈ [0, 200] °C and < peak_temp_c
//   DFM-TEMPER-STRESS  : surface_stress_mpa ∈ [40, 200] MPa  (info if < 69)
//
// Recognize: metadata-only.  Residual stress can be probed with a polarimeter,
// but B-Rep alone cannot reveal it.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace temper_glass {

constexpr const char* kSkillId = "temper_glass";

struct Input
{
    double  peak_temp_c         = 650.0;
    double  quench_temp_c       = 20.0;
    double  surface_stress_mpa  = 100.0;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace temper_glass
}  // namespace koocadcam::skill
