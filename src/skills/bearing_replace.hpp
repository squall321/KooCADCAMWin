#pragma once
// @lat: [[engine/skills#bearing_replace]]
//
// bearing_replace — maintenance record for swapping a worn rolling-element
// bearing on a rotating shaft.  The bearing itself is a sub-OCCT subsystem
// (race + balls + cage + seals) that is NOT modelled as B-rep; this skill
// only stamps the installation record so downstream MES / CMMS can audit
// the maintenance event and the next planner can verify the seat fit.
//
//        ─────┬───────────────────┬─────
//             │  ╱╲ ╱╲ ╱╲ ╱╲ ╱╲  │   ← bearing (outer race + rollers + cage)
//             │  ╲╱ ╲╱ ╲╱ ╲╱ ╲╱  │
//        ─────┴───────────────────┴─────
//                       ↑
//                  bearing_id 6205-2RS, location "MOTOR-DE"
//
// Signature pattern:
//   - pattern.kind             = "bearing_replace"
//   - pattern.bearing_id       = catalog number (e.g. "6205-2RS")
//   - pattern.location         = human label (e.g. "MOTOR-DE", "GEARBOX-INPUT")
//   - pattern.install_torque_nm
//   - pattern.geometry_changed = false
//
// DFM checks:
//   bearing_id      non-empty                (DFM-INPUT error)
//   location        non-empty                (DFM-INPUT error)
//   install_torque_nm > 0                    (DFM-INPUT error if ≤ 0)
//   install_torque_nm ∈ [5, 500] Nm          (DFM-BEARING-TORQUE info if outside)
//
// Synthesis:
//   NO geometric change — bearing is a sub-OCCT part.  Clone shape +
//   material and stamp the maintenance signature.
//
// Recognize:
//   Metadata-only replay from `workpiece.features()`.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bearing_replace {

constexpr const char* kSkillId = "bearing_replace";

struct Input
{
    std::string bearing_id       = "6205-2RS";
    std::string location         = "MOTOR-DE";
    double      install_torque_nm = 50.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bearing_replace
}  // namespace koocadcam::skill
