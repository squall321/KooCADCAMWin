#pragma once
// @lat: [[engine/skills#drill_and_tap]]
//
// drill_and_tap — COMPOUND MACRO that drills a pilot hole and (logically)
// taps an internal thread inside it.
//
//                        ┌─ entry_face (planar)
//                        ▼
//                  ──────┬──────────┬──────
//                        │ pilot │           ↑ pilot_extra_depth_mm
//                        │ dia   │           │   (extra clearance for chips)
//                        │       │           ↓
//                        │ ◄►    │           ↑ tap_depth_mm
//                        │       │           │
//                        │       │           ↓
//                        └───────┘
//                            ↑
//                       pilot_dia (from thread_size table)
//
// The total drilled depth = tap_depth_mm + pilot_extra_depth_mm so chips
// have somewhere to go below the tapped section.
//
// Thread → pilot diameter table (metric, ISO 68-1 coarse pitch, common sizes):
//   M1.6 → 1.25,  M2 → 1.6,  M2.5 → 2.05,  M3 → 2.5,
//   M4   → 3.3,   M5 → 4.2,  M6   → 5.0.
//
// Implementation strategy (slice-1):
//   1. Look up pilot_dia from thread_size in the hardcoded table.
//   2. Compose `drill_hole` with diameter=pilot_dia, depth=tap_depth_mm +
//      pilot_extra_depth_mm.  (If through_hole=true, depth is ignored.)
//   3. Tap geometry itself is NOT yet modeled (thread helical surface is
//      defer-modelled).  We only record tap METADATA on the signature so
//      CAPP can schedule the tap operation.
//
// Signature pattern (what the skill leaves):
//   - Same topology as a plain drill_hole (1 cyl face + bounding circles)
//     of diameter = pilot_dia.
//   - signature.skill_id == "drill_and_tap"
//   - signature.pattern.kind == "drill_and_tap"
//   - signature.pattern.thread_size and signature.pattern.tap_depth_mm
//     carry the tap-specific metadata that recognize() can use to recover
//     the original thread spec.
//   - signature.tooling.tool_type == "drill;tap" (sequence) with the
//     individual tools described in signature.tooling.extra.tool_sequence.
//
// DFM checks:
//   DFM-002      : pilot_dia ≥ 0.8 mm (mirrored from drill_hole).
//   DFM-TAP-SIZE : thread_size unknown → error.
//   DFM-TAP-DEPTH: tap_depth_mm ≤ 0 → error.
//   Pilot-depth/dia ratio (= drill_hole's DFM-PECK) is checked through
//   the inner call.
//
// Recognize:
//   Look at every drill_hole candidate whose recovered diameter matches a
//   standard pilot diameter (± 0.05 mm).  Reverse-lookup the thread_size
//   from the table.  Confidence ≈ 0.6 (overlaps with a plain drill_hole
//   that happens to have a "standard" pilot diameter — CAPP may need
//   user input to disambiguate).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace drill_and_tap {

constexpr const char* kSkillId = "drill_and_tap";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm        = 0.0;
    double      position_y_mm        = 0.0;
    gp_Dir      axis_dir             { 0.0, 0.0, -1.0 };
    std::string thread_size;                 // "M3", "M4", …
    double      tap_depth_mm         = 0.0;  // active thread length
    double      pilot_extra_depth_mm = 1.0;  // extra drill below the tap for chips
    bool        through_hole         = false;
};

// Resolve thread → pilot diameter.  Returns 0.0 if size unknown.
double pilotDiameterFor(const std::string& thread_size);

// Reverse-resolve pilot diameter → thread size (or "" if no match).
std::string threadSizeFor(double pilot_dia_mm, double tol_mm = 0.05);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace drill_and_tap
}  // namespace koocadcam::skill
