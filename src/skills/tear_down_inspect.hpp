#pragma once
// @lat: [[engine/skills#tear_down_inspect]]
//
// tear_down_inspect — controlled disassembly of an assembly for inspection.
// Each part is separated, cleaned, and visually / dimensionally surveyed.
// Steps are logged so a downstream reassembly skill can follow the inverse
// order.
//
//        ┌──────────────┐
//        │  assembly    │   tear_down_inspect             ┌──── part_A
//        │  (n parts)   │   ──────────────────────▶       ├──── part_B
//        │  bolted /    │   (steps_taken, time_min,       ├──── part_C
//        │  press-fit)  │    parts_marked)                └──── ...
//        └──────────────┘
//          shape identical;  inspection metadata stamped
//
// This is a maintenance-shop bookkeeping skill — no OCCT B-rep change.
// Inspection findings are routed via DFMReport (info-level by default;
// "error" only when the operator-supplied counts contradict each other,
// e.g. steps_taken = 0 with parts_marked > 0).
//
// Signature pattern:
//   - pattern.kind             = "tear_down_inspect"
//   - pattern.steps_taken      = <int>
//   - pattern.time_min         = <double>
//   - pattern.parts_marked     = <int>
//   - pattern.inspection_pass  = <bool>      (parts_marked == 0 ⇒ true)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT       steps_taken ≤ 0 (error) — at least one disassembly step
//   DFM-INPUT       time_min ≤ 0    (error)
//   DFM-INPUT       parts_marked < 0 (error)
//   DFM-TDI-CONSIST parts_marked > 0 with steps_taken == 1 (info — flag a
//                   single-step teardown that nevertheless found defects)
//   DFM-TDI-LONG    time_min > 480 (info — >8 h teardown is unusual)
//
// Synthesis:
//   NO geometric change.  Clones the workpiece and stamps the signature.
//
// Recognize:
//   Metadata-only replay (assembly history cannot be inferred from a single
//   shape's B-rep).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tear_down_inspect {

constexpr const char* kSkillId = "tear_down_inspect";

struct Input
{
    int     steps_taken   = 1;
    double  time_min      = 15.0;
    int     parts_marked  = 0;       // count of parts flagged during inspection
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tear_down_inspect
}  // namespace koocadcam::skill
