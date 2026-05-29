#pragma once
// @lat: [[engine/skills#interior_install]]
//
// interior_install — automotive trim line station that installs interior
// modules (dashboard, IP, headliner, seats, door panels) and the wiring
// harness onto the vehicle.  Records component count and total routed
// harness length.  Macroscopic B-rep on the parent workpiece is unchanged;
// only the assembly record is stamped.
//
//       ┌────────────────────────────────────┐
//       │  body shell        seats × N       │
//       │  IP/dash  ───────────── harness ── │
//       │  headliner            (length_m)   │
//       │  door panels                       │
//       └────────────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "interior_install"
//   - pattern.component_count  = int (trim modules installed)
//   - pattern.harness_length_m = total routed harness (m)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   component_count > 0                                (DFM-INPUT error if <= 0)
//   harness_length_m > 0                               (DFM-INPUT error if <= 0)
//   harness_length_m > 200                             (DFM-INTERIOR-HARNESS info)
//   component_count > 100                              (DFM-INTERIOR-COMPONENT info)
//
// Synthesis:
//   NO geometric change.  Clone shape + material, stamp signature.
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

namespace interior_install {

constexpr const char* kSkillId = "interior_install";

struct Input
{
    int     component_count  = 25;     // interior trim modules
    double  harness_length_m = 60.0;   // total routed wiring harness (m)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace interior_install
}  // namespace koocadcam::skill
