#pragma once
// @lat: [[engine/skills#peel_test]]
//
// peel_test — destructive adhesive peel test (ASTM D903 / D1876 family)
// record.  A flexible substrate bonded to a rigid (or flexible)
// counter-substrate is peeled at a defined angle (typically 90° or
// 180°) and the peel force per unit width is recorded.  Macroscopic
// CAD geometry of the parent workpiece is unchanged.
//
//             ▲ peel
//             │   (peel_angle_deg)
//          ───┴───
//          │ tape │
//      ════╪══════╪═══════════ ← bond line / adhesive_type
//          │ part │
//          ───────
//
// Signature pattern:
//   - pattern.kind             = "peel_test"
//   - pattern.peel_angle_deg
//   - pattern.peel_force_n_mm
//   - pattern.adhesive_type    ("PSA"|"epoxy"|"cyanoacrylate"|"silicone"|…)
//   - pattern.bond_quality     ("weak"|"nominal"|"strong")
//   - pattern.geometry_changed = false
//
// DFM checks:
//   peel_angle_deg   ∈ (0, 180]              (DFM-PEEL-ANGLE error if out)
//   peel_force_n_mm  > 0                     (DFM-INPUT error)
//   adhesive_type    non-empty               (DFM-INPUT warning)
//   peel_force_n_mm  ≤ 100                   (DFM-PEEL-FORCE info)

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace peel_test {

constexpr const char* kSkillId = "peel_test";

struct Input
{
    double      peel_angle_deg  = 90.0;          // deg
    double      peel_force_n_mm = 1.5;           // N/mm
    std::string adhesive_type   = "PSA";         // "PSA|epoxy|cyanoacrylate|silicone"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace peel_test
}  // namespace koocadcam::skill
