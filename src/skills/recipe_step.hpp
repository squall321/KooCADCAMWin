#pragma once
// @lat: [[engine/skills#recipe_step]]
//
// recipe_step — single process-recipe step record.  A recipe is a sequence
// of (parameter_name, target_value ± tolerance) entries that the operator /
// PLC drives to during a stage of the run.  This skill stamps ONE such
// step onto the workpiece feature history; multiple invocations make up
// the full recipe.
//
//                step_id=12: pressure_psi → 145.0 ± 2.5
//                step_id=13: temp_c       →  82.0 ± 0.5
//                step_id=14: dwell_s      →  30.0 ± 1.0
//
// Signature pattern:
//   - pattern.kind           = "recipe_step"
//   - pattern.step_id        = <input>
//   - pattern.parameter_name = <input>
//   - pattern.target_value   = <input>
//   - pattern.tolerance      = <input>
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT          step_id < 0                  → error
//   DFM-INPUT          parameter_name empty         → error
//   DFM-INPUT          tolerance < 0                → error
//   DFM-RECIPE-TOL     tolerance > |target_value|   → info (loose tolerance)
//
// Synthesis:
//   NO geometric change — clone shape + material verbatim, copy features,
//   append signature.
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

namespace recipe_step {

constexpr const char* kSkillId = "recipe_step";

struct Input
{
    int          step_id        = 0;
    std::string  parameter_name = "";
    double       target_value   = 0.0;
    double       tolerance      = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace recipe_step
}  // namespace koocadcam::skill
