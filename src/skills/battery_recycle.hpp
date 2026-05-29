#pragma once
// @lat: [[engine/skills#battery_recycle]]
//
// battery_recycle — end-of-life battery recovery operation.  Spent cells of
// a stated chemistry (Li-ion, NiMH, Pb-acid) are routed through shredding,
// pyrometallurgical or hydrometallurgical processing to recover cathode
// metals (Co, Ni, Li, Mn) at a quoted mass-recovery percentage.  This is
// pure circular-economy metadata: the workpiece B-rep is unchanged; only
// the recovery record is stamped so a downstream materials accounting
// module can credit recovered mass against virgin feedstock.
//
//          ┌────────────────┐    battery_recycle    ┌────────────────┐
//          │ spent cell EOL │  ──────────────────▶ │ stamp record   │
//          │ mass_kg, chem  │ (chem, recovery_pct)  │ recovery_pct   │
//          └────────────────┘                       └────────────────┘
//             geometry identical;  metadata-only operation
//
// Signature pattern:
//   - pattern.kind              = "battery_recycle"
//   - pattern.battery_chemistry = "Li-ion" | "NiMH" | "Pb-acid"
//   - pattern.mass_kg
//   - pattern.recovery_pct
//   - pattern.recovered_mass_kg = mass_kg * recovery_pct / 100
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   battery_chemistry non-empty                  (DFM-INPUT error)
//   chemistry ∈ {Li-ion, NiMH, Pb-acid}          (DFM-BATT-CHEM info if unknown)
//   mass_kg       > 0                            (DFM-INPUT error)
//   recovery_pct  ∈ (0, 100]                     (DFM-INPUT error)
//   recovery_pct  ≤ 95 typical ceiling           (DFM-BATT-RECOVERY info if exceeded)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
//
// Recognize:
//   Metadata-only — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace battery_recycle {

constexpr const char* kSkillId = "battery_recycle";

struct Input
{
    std::string  battery_chemistry = "Li-ion";   // "Li-ion" | "NiMH" | "Pb-acid"
    double       mass_kg           = 10.0;       // spent cell pack mass
    double       recovery_pct      = 85.0;       // recovered cathode metal fraction
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace battery_recycle
}  // namespace koocadcam::skill
