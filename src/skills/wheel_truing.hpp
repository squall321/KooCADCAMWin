#pragma once
// @lat: [[engine/skills#wheel_truing]]
//
// wheel_truing — under-floor wheel-lathe re-profiling of railway wheels to
// restore the tread, flange and rolling diameter to specification.  Whilst
// a real lathe DOES remove metal, KooCADCAM models this as a metadata-only
// signature: the macroscopic wheel B-rep is not parametrised at this level
// and the truing record (dia after re-profile, flange height, run-out) is
// what downstream maintenance / wear forecasting consume.
//
// Signature pattern:
//   - pattern.kind              = "wheel_truing"
//   - pattern.wheel_dia_mm      = diameter after re-profile (mm)
//   - pattern.flange_height_mm  = restored flange height (mm)
//   - pattern.run_out_mm        = measured radial run-out after truing (mm)
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   wheel_dia_mm     ∈ [780, 1250]            (DFM-WT-DIA error if outside)
//   flange_height_mm ∈ [22, 36]               (DFM-WT-FLANGE error if outside)
//   run_out_mm       ∈ [0, 0.3]               (DFM-WT-RUNOUT error if too high)
//   wheel_dia_mm < 840  (scrap limit warning) (DFM-WT-SCRAP info)
//
// Synthesis:
//   NO geometric change in this model. Clone + stamp.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wheel_truing {

constexpr const char* kSkillId = "wheel_truing";

struct Input
{
    double wheel_dia_mm     = 920.0;   // diameter after re-profile
    double flange_height_mm = 28.0;    // restored flange height
    double run_out_mm       = 0.05;    // measured radial run-out
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wheel_truing
}  // namespace koocadcam::skill
