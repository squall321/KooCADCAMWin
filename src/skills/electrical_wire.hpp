#pragma once
// @lat: [[engine/skills#electrical_wire]]
//
// electrical_wire — installation record for a building electrical
// wiring run (conductor + insulation).  Conductors are sub-OCCT
// routing entities; the host workpiece geometry stays unchanged and
// the wiring is captured as metadata for ampacity / voltage-drop /
// circuit checks.
//
// Signature pattern:
//   - pattern.kind             = "electrical_wire"
//   - pattern.conductor_dia_mm = bare conductor Ø
//   - pattern.length_m         = run length
//   - pattern.voltage_v        = nominal circuit voltage
//   - pattern.circuit_count    = parallel circuits in the run
//   - pattern.geometry_changed = false
//
// DFM checks:
//   conductor_dia_mm ∈ (0, 50]               (DFM-WIRE-DIA error)
//   length_m         > 0                     (DFM-INPUT error)
//   voltage_v        ∈ (0, 35000]            (DFM-WIRE-VOLT error)
//   circuit_count    ≥ 1                     (DFM-INPUT error)
//   voltage_v > 1000 && conductor_dia_mm < 5 (DFM-WIRE-INSUL info, thin HV)

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace electrical_wire {

constexpr const char* kSkillId = "electrical_wire";

struct Input
{
    double conductor_dia_mm = 2.6;
    double length_m         = 20.0;
    double voltage_v        = 230.0;
    int    circuit_count    = 1;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace electrical_wire
}  // namespace koocadcam::skill
