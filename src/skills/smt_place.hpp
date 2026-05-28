#pragma once
// @lat: [[engine/skills#smt_place]]
//
// smt_place — Surface-Mount Technology component placement.  A pick-and-place
// machine deposits SMD components (resistors, capacitors, ICs, …) onto a PCB
// pre-loaded with solder paste.  This skill is a process-record placeholder:
// the macroscopic PCB B-rep is UNCHANGED.  What is recorded is the
// placement run (count of components, machine throughput, accuracy spec).
//
//     ┌────────────────────────────────────────────┐
//     │  □ □ □     ▣ ▣      ◫ ◫ ◫     ▤             │  ← SMD components seated
//     │            ▣ ▣                              │    on solder paste
//     │  □ □ □     ▣ ▣      ◫ ◫ ◫     ▤             │
//     └────────────────────────────────────────────┘  ← PCB substrate
//
// Signature pattern:
//   - pattern.kind                 = "smt_place"
//   - pattern.component_count      = <input>
//   - pattern.placement_rate_cph   = <input>   (components per hour)
//   - pattern.accuracy_um          = <input>
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   DFM-INPUT              component_count  ≤ 0 (error)
//   DFM-INPUT              placement_rate_cph ≤ 0 (error)
//   DFM-INPUT              accuracy_um ≤ 0 (error)
//   DFM-SMT-ACCURACY       accuracy_um > 100 μm (error — coarser than 01005 spec)
//   DFM-SMT-RATE           placement_rate_cph > 200000 (info — above any
//                          industrial machine spec)
//
// Recognize:
//   Impossible from B-rep.  Returns ONLY signatures replayed from
//   `workpiece.features()` filtered by skill_id — confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace smt_place {

constexpr const char* kSkillId = "smt_place";

struct Input
{
    int     component_count      = 0;       // total parts placed in the run
    double  placement_rate_cph   = 0.0;     // components per hour (machine throughput)
    double  accuracy_um          = 50.0;    // placement accuracy spec (μm, typ. 25-50)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace smt_place
}  // namespace koocadcam::skill
