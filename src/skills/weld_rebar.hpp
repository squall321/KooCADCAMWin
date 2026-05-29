#pragma once
// @lat: [[engine/skills#weld_rebar]]
//
// weld_rebar — tack or full structural weld of reinforcing bar joints
// (lap, butt, or cross-tie at intersections of a rebar cage).  AWS D1.4
// governs the qualified procedures; phase-1 KooCADCAM records the joint
// type and count so downstream BIM / QA can verify NDE coverage.  The
// underlying B-rep is untouched — the bars themselves are out-of-model.
//
//       ──────────────────              ──────────────────
//       ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒                     ╲╱
//       ──────────────────  ←  lap weld;     ╱╲   ← cross-tie tack
//                                    or                          (where
//                                                                 verticals
//                                                                 cross
//                                                                 horizontals)
//
// Signature pattern:
//   - pattern.kind             = "weld_rebar"
//   - pattern.weld_type        = "tack" | "full" | "lap" | "butt"
//   - pattern.joint_count      = number of joints welded in this batch
//   - pattern.geometry_changed = false
//
// DFM checks:
//   joint_count ≥ 1                              (DFM-INPUT error)
//   weld_type ∈ {"tack","full","lap","butt"}     (DFM-WELD-TYPE error)
//   tack on full structural splice ⇒ undersized   (DFM-WELD-TACK info)
//   joint_count > 500 in one batch ⇒ stage      (DFM-WELD-BATCH info)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace weld_rebar {

constexpr const char* kSkillId = "weld_rebar";

struct Input
{
    std::string  weld_type    = "tack";    // "tack" | "full" | "lap" | "butt"
    int          joint_count  = 1;         // total joints in this batch
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace weld_rebar
}  // namespace koocadcam::skill
