#pragma once
// @lat: [[engine/skills#ic_bond]]
//
// ic_bond — Semiconductor die-to-package interconnect formation.  After a die
// is attached, electrical contact is formed between the die's bond pads and
// the package leadframe / substrate by thin wires (gold/copper ball bonds,
// aluminum wedge bonds) or by ultrasonic flip-chip stitches.  The wires are
// sub-100-μm in diameter; the macroscopic package B-rep is UNCHANGED.  What
// is recorded is the bonding recipe — bond type, count, ultrasonic power.
//
//                  □  →  loop  →  □       ← wire bond (Au/Cu ball; Al wedge)
//                  □              □
//        ┌─────────┴──────────────┴──────────┐
//        │ ▓▓▓  semiconductor die (Si)  ▓▓▓ │
//        ├──────────────────────────────────┤
//        │  package substrate / leadframe   │
//        └──────────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind                 = "ic_bond"
//   - pattern.bond_type            = "ball" | "wedge" | "stitch" | "ribbon"
//   - pattern.bond_count           = <input>
//   - pattern.ultrasonic_power_w   = <input>
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   DFM-INPUT              bond_count ≤ 0                  (error)
//   DFM-INPUT              ultrasonic_power_w < 0          (error)
//   DFM-BOND-TYPE          bond_type not in catalog        (error)
//   DFM-BOND-USPOWER-HIGH  ultrasonic_power_w > 5.0 W      (error — Si die crack risk)
//   DFM-BOND-USPOWER-LOW   ultrasonic_power_w < 0.05 W     (info — non-stick risk)
//   DFM-BOND-COUNT-HIGH    bond_count > 5000               (info — long cycle time)
//
// Recognize:
//   Impossible from B-rep (wires sub-OCCT).  Returns ONLY signatures replayed
//   from `workpiece.features()` filtered by skill_id — confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ic_bond {

constexpr const char* kSkillId = "ic_bond";

struct Input
{
    std::string  bond_type           = "ball";   // "ball" | "wedge" | "stitch" | "ribbon"
    int          bond_count          = 0;
    double       ultrasonic_power_w  = 0.5;      // typical 0.1 – 2.0 W
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ic_bond
}  // namespace koocadcam::skill
