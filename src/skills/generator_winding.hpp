#pragma once
// @lat: [[engine/skills#generator_winding]]
//
// generator_winding — stator/rotor coil winding installation record for a
// rotating electrical machine (alternator, traction motor).  Insulated
// magnet wire is wound through stator slots; turns-per-coil and conductor
// diameter set fill factor & resistance.  The macroscopic core lamination
// stack is unchanged; this skill stamps the winding metadata.
//
//   slot 1: ▓▓▓▓  slot 2: ▓▓▓▓  …   turns wrapped in series → phase coil
//
// Signature pattern:
//   - pattern.kind              = "generator_winding"
//   - pattern.conductor_dia_mm  = magnet-wire bare Ø
//   - pattern.slot_count        = number of stator slots wound
//   - pattern.turns_per_coil    = effective turns per coil
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   conductor_dia_mm ∈ (0.05, 5.0]    (DFM-WIND-DIA error if outside)
//   slot_count       > 0              (DFM-INPUT error if ≤ 0)
//   turns_per_coil   > 0              (DFM-INPUT error if ≤ 0)
//
// Synthesis: no geometric change — clone + stamp.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace generator_winding {

constexpr const char* kSkillId = "generator_winding";

struct Input
{
    double conductor_dia_mm = 1.0;   // bare Ø
    int    slot_count       = 24;
    int    turns_per_coil   = 50;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace generator_winding
}  // namespace koocadcam::skill
