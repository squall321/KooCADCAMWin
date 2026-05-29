#pragma once
// @lat: [[engine/skills#disassemble_step]]
//
// disassemble_step — controlled disassembly of an assembly into sub-parts at
// end-of-life.  The skill records WHAT was disassembled (step count, fastener
// count removed, time spent) without changing the workpiece B-rep — the
// physical separation happens off-OCCT (on the line) and the result is
// described, not synthesized.
//
//        ┌──────────────────┐                       ┌──────────────────┐
//        │  assembly (EOL)  │  disassemble_step   │  assembly (EOL)  │
//        │  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒  │  ────────────────▶ │  (signature only)│
//        │  ▒▒▒ fasteners ▒▒│  (N steps, k fast,   │  step_count, …   │
//        │  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒  │   t minutes)         │                  │
//        └──────────────────┘                       └──────────────────┘
//          shape unchanged;  metadata records the sequence
//
// Signature pattern:
//   - pattern.kind                = "disassemble_step"
//   - pattern.step_count          = number of disassembly operations
//   - pattern.fasteners_removed   = bolts / screws / clips removed
//   - pattern.time_min            = wall-clock disassembly time (min)
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   step_count        > 0       (DFM-INPUT error if not)
//   fasteners_removed ≥ 0       (DFM-INPUT error if negative)
//   time_min          > 0       (DFM-INPUT error if not)
//   time_min ≥ 120 → DFM-DISASM-TIME warning (industry rule-of-thumb:
//                    > 2 h disassembly cost overruns scrap-value)
//   step_count > 50 → DFM-DISASM-COMPLEX info (design-for-disassembly hint)
//
// Synthesis:
//   NO geometric change.  Clone the workpiece (shape + material) + stamp.
//
// Recognize:
//   Impossible from B-rep alone.  Returns ONLY signatures replayed from
//   `workpiece.features()` filtered by this skill_id — confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace disassemble_step {

constexpr const char* kSkillId = "disassemble_step";

struct Input
{
    int     step_count          = 1;
    int     fasteners_removed   = 0;
    double  time_min            = 1.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace disassemble_step
}  // namespace koocadcam::skill
