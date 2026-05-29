#pragma once
// @lat: [[engine/skills#cell_formation]]
//
// cell_formation — first-cycle charge/discharge (and subsequent aging) that
// grows the solid-electrolyte interphase (SEI) on the anode and stabilises
// cathode lattice cycling.  This is mandatory for every Li-ion cell.  After
// formation the cell is graded by recovered capacity, then aged in
// temperature-controlled racks to allow voltage to settle.
//
//        ┌── current ──┐
//        │             │
//   ─────┴──┐    ┌─────┴─────  charge/discharge profile
//          ╲╱    ╲╱           (typ. C/20 → C/10 → C/3 over several cycles)
//
// Capacity recovered after formation is THE acceptance criterion for a cell.
// Process owners track {charge_rate_c, cycle_count, target_capacity_mAh}.
//
// Signature pattern:
//   - pattern.kind                  = "cell_formation"
//   - pattern.charge_rate_c         = formation C-rate (e.g. 0.05 C, 0.1 C)
//   - pattern.cycle_count           = number of formation cycles
//   - pattern.target_capacity_mah   = nameplate cell capacity (mAh)
//   - pattern.upper_voltage_v       = upper cutoff (typ. 4.2 V for NMC, 3.65 V for LFP)
//   - pattern.aging_days            = post-formation rest period
//   - pattern.geometry_changed      = false
//
// DFM checks:
//   DFM-INPUT             charge_rate_c ≤ 0 or target_capacity_mah ≤ 0 (error)
//   DFM-FORM-CYCLES       cycle_count < 1 (error) or cycle_count > 20 (info)
//   DFM-FORM-RATE-FAST    charge_rate_c > 0.5 (info — SEI quality risk)
//   DFM-FORM-RATE-SLOW    charge_rate_c < 0.02 (info — line throughput hit)
//   DFM-FORM-VOLT         upper_voltage_v outside [2.0, 4.5] V (info)
//   DFM-FORM-AGING        aging_days < 0 (error); > 30 (info)
//
// Synthesis:
//   NO geometric change — clone and stamp the signature.
//
// Recognize:
//   Metadata-only replay, confidence 1.0.  SEI is sub-OCCT chemistry.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cell_formation {

constexpr const char* kSkillId = "cell_formation";

struct Input
{
    double  charge_rate_c        = 0.1;     // C/10 nominal formation
    int     cycle_count          = 3;       // typ. 3 - 5 formation cycles
    double  target_capacity_mah  = 3000.0;  // ~ 18650 NMC nameplate
    double  upper_voltage_v      = 4.2;     // NMC cutoff
    double  aging_days           = 14.0;    // calendar aging post-formation
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cell_formation
}  // namespace koocadcam::skill
