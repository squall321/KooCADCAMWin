#pragma once
// @lat: [[engine/skills#replace_part]]
//
// replace_part — swap one or more parts in an assembly.  Each old part is
// pulled out and a new part is installed in the same nominal location.
// The skill records the pairing (old ↔ new id lists) plus the labour time,
// so a maintenance log can be reconstructed.
//
//        ┌──────────┐                                ┌──────────┐
//        │ assembly │   replace_part                 │ assembly │
//        │   ╱╲ A   │   ───────────────────▶          │   ╱╲ A   │
//        │  ╱  ╲ B  │   (old: [B,C], new: [B', C'])   │  ╱  ╲ B' │
//        │ ╱    ╲ C │                                 │ ╱    ╲ C'│
//        └──────────┘                                └──────────┘
//          shape identical at slice-1 granularity;  swap recorded as metadata
//
// In slice-1 we do not change the OCCT B-rep — the assembly's outer shell
// is preserved (replacement parts are assumed dimensionally interchangeable;
// "form-fit-function" identical).  The swap is recorded as metadata only.
//
// Signature pattern:
//   - pattern.kind             = "replace_part"
//   - pattern.old_part_ids     = [ ... ]
//   - pattern.new_part_ids     = [ ... ]
//   - pattern.part_count       = old_part_ids.size()
//   - pattern.work_time_min    = <double>
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT      old_part_ids empty                       (error)
//   DFM-INPUT      new_part_ids empty                       (error)
//   DFM-INPUT      work_time_min ≤ 0                        (error)
//   DFM-REPL-COUNT old_part_ids.size() != new_part_ids.size() (error —
//                  every removed part must be replaced 1:1)
//   DFM-REPL-DUP   duplicated id within old_part_ids        (info)
//   DFM-REPL-LONG  work_time_min > 240                      (info)
//
// Synthesis:
//   NO geometric change.  Clones the workpiece and stamps the signature.
//
// Recognize:
//   Metadata-only replay.  Replacement events leave no B-rep trace once the
//   replacement parts have the same outer envelope.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace replace_part {

constexpr const char* kSkillId = "replace_part";

struct Input
{
    std::vector<std::string>  old_part_ids;
    std::vector<std::string>  new_part_ids;
    double                    work_time_min = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace replace_part
}  // namespace koocadcam::skill
