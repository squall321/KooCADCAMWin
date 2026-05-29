#pragma once
// @lat: [[engine/skills#turbine_balance]]
//
// turbine_balance — high-speed rotor dynamic-balance correction record.
// A turbine rotor (gas, steam, hydro, micro-jet) is spun on a balancing
// machine; vibration phase & amplitude reveal mass imbalance which is
// then corrected by trim weights / material removal at planes.  The
// macroscopic B-rep is unchanged (trim mass is sub-OCCT); the rotor's
// dynamic-balance class is updated to e.g. ISO 21940 G1.0.
//
//   imbalance vector (g·mm) at plane A & B  ──▶   trim weights / grinding
//   ─────────────────────────────────────         ─────────────────────────
//
// Signature pattern:
//   - pattern.kind                = "turbine_balance"
//   - pattern.rotor_id            = <input>
//   - pattern.mass_imbalance_g_mm = residual (target ≤ permissible)
//   - pattern.balance_grade       = "G0.4" | "G1.0" | "G2.5" | "G6.3"
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   rotor_id           non-empty                          (DFM-INPUT error)
//   mass_imbalance_g_mm ≥ 0                               (DFM-INPUT error)
//   balance_grade ∈ {G0.4,G1.0,G2.5,G6.3,G16}             (DFM-BAL-GRADE info)
//
// Synthesis: no geometric change — clone + stamp signature.
// Recognize: metadata-only (mass distribution is sub-OCCT).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace turbine_balance {

constexpr const char* kSkillId = "turbine_balance";

struct Input
{
    std::string rotor_id            = "R-001";
    double      mass_imbalance_g_mm = 5.0;    // residual after trim (g·mm)
    std::string balance_grade       = "G2.5"; // ISO 21940 grade
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace turbine_balance
}  // namespace koocadcam::skill
