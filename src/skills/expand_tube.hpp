#pragma once
// @lat: [[engine/skills#expand_tube]]
//
// expand_tube — plastically expand a tube end inside a tubesheet bore so the
// tube wall grips the tubesheet hole and (optionally) seals against it.  The
// expansion can be HYDRAULIC (pressurised mandrel, very uniform) or MECHANICAL
// (rolled-in expander with three rollers, faster but more variable).
//
//   BEFORE:   ┌──── tubesheet ────┐         AFTER:   ┌──── tubesheet ────┐
//             │  ╔══ bore ══╗     │                  │  ╔══ bore ══╗     │
//             │  ║  ┌────┐  ║     │                  │  ║┌────────┐║     │
//             │  ║  │tube│  ║     │                  │  ║│ tube   │║     │
//             │  ║  └────┘  ║     │                  │  ║│ wall   │║     │
//             │  ╚══════════╝     │                  │  ║│ pressed│║     │
//             │  (clearance gap)  │                  │  ║│ against│║     │
//             └───────────────────┘                  │  ║└────────┘║     │
//                                                    │  ╚══════════╝     │
//                                                    └───────────────────┘
//
// Signature pattern:
//   - pattern.kind                  = "expand_tube"
//   - pattern.target_expansion_pct  (5 – 12 % typical; > 15 % overworks tube)
//   - pattern.method                = "hydraulic" | "mechanical"
//   - pattern.geometry_changed      = false
//
// DFM checks:
//   DFM-INPUT             target_expansion_pct ≤ 0
//   DFM-EXPAND-RANGE      < 2 % or > 15 % → bad grip / overstrain (error)
//   DFM-EXPAND-METHOD     method ∉ {hydraulic, mechanical} (info)
//
// Synthesis:
//   Geometric no-op for slice-1.  The B-rep ID of the tube does not change;
//   we record the expansion ratio + method so downstream process planning
//   (pull-out test, hardness check) knows the as-installed state.
//
// Recognize:
//   Metadata-only replay.

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace expand_tube {

constexpr const char* kSkillId = "expand_tube";

struct Input
{
    double      target_expansion_pct = 8.0;          // % ID increase
    std::string method                = "hydraulic"; // "hydraulic" | "mechanical"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace expand_tube
}  // namespace koocadcam::skill
