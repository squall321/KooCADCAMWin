#pragma once
// @lat: [[engine/skills#motor_winding_ev]]
//
// motor_winding_ev — record an EV traction-motor stator winding operation.
// Copper hairpins or distributed wire are inserted into the stator slots; the
// macroscopic stator B-rep is not changed by this skill (the winding is a
// sub-OCCT entity).  This skill stamps the winding-process record so MES /
// quality / process-planning can downstream verify slot fill and dispatch a
// resistance / hi-pot test.
//
//          stator OD ─────────────────────────────╮
//         ┌──────────────────────────────────────┐│
//         │ ░░ slot[0]  ░░ slot[1]  ░░ slot[2] … ││  ← slot_count
//         │ ░░ copper   ░░ copper   ░░ copper    ││  ← copper_fill_pct
//         └──────────────────────────────────────┘│
//          stator ID ─────────────────────────────╯
//
// Signature pattern:
//   - pattern.kind             = "motor_winding_ev"
//   - pattern.stator_id        = <input>
//   - pattern.slot_count       = <input>
//   - pattern.copper_fill_pct  = <input>
//   - pattern.geometry_changed = false
//
// DFM checks:
//   stator_id non-empty                       (DFM-INPUT error)
//   slot_count ∈ [12, 96] (typical EV)        (DFM-WIND-SLOT info if outside)
//   copper_fill_pct ∈ (0, 75]                 (DFM-WIND-FILL error if out)
//
// Synthesis: NO geometric change. Clone shape + material, copy features, stamp.
// Recognize: metadata-only replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace motor_winding_ev {

constexpr const char* kSkillId = "motor_winding_ev";

struct Input
{
    std::string  stator_id        = "STATOR-EV-001";
    int          slot_count       = 48;       // typical 3-phase EV: 36 / 48 / 72
    double       copper_fill_pct  = 55.0;     // slot copper area fraction (%)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace motor_winding_ev
}  // namespace koocadcam::skill
