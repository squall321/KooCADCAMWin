#pragma once
// @lat: [[engine/skills#fuel_cell_stack_assemble]]
//
// fuel_cell_stack_assemble — stack-up of PEM / SOFC / AFC fuel cells into a
// multi-cell stack.  Each cell (MEA: membrane-electrode assembly) is
// sandwiched between bipolar flow plates and clamped between two end
// plates with tie rods.  At the CAD level the part list is recorded but
// the workpiece geometry stays as-is for assembly bookkeeping.  Stack
// open-circuit voltage equals cell_count × per-cell V (≈ 1.0 V for PEM,
// ≈ 0.9 V for SOFC).
//
//        [ end ][ MEA × N ][ flow plate × N+1 ][ end ]   tie rods × 4
//        ─────────────────────────────────────────────
//                V_stack  =  N × V_cell
//
// Signature pattern:
//   - pattern.kind             = "fuel_cell_stack_assemble"
//   - pattern.cell_count       = N
//   - pattern.mea_type         = "PEM" | "SOFC" | "AFC" | "PAFC" | "MCFC"
//   - pattern.stack_voltage_v
//   - pattern.geometry_changed = false
//
// DFM checks:
//   cell_count ≥ 1 (integer)                     (DFM-INPUT error)
//   cell_count ≤ 600                             (DFM-FC-COUNT info — large stack)
//   mea_type non-empty                           (DFM-INPUT warning)
//   mea_type in known set                        (DFM-FC-MEA info)
//   stack_voltage_v > 0                          (DFM-INPUT error)
//   stack_voltage_v ≈ cell_count × per-cell V    (DFM-FC-VOLT info if mismatch ≥ 20 %)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
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

namespace fuel_cell_stack_assemble {

constexpr const char* kSkillId = "fuel_cell_stack_assemble";

struct Input
{
    int          cell_count      = 100;
    std::string  mea_type        = "PEM";   // "PEM" | "SOFC" | "AFC" | "PAFC" | "MCFC"
    double       stack_voltage_v = 70.0;    // measured / target stack OCV
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fuel_cell_stack_assemble
}  // namespace koocadcam::skill
