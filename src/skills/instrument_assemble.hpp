#pragma once
// @lat: [[engine/skills#instrument_assemble]]
//
// instrument_assemble — metadata-only record that a musical instrument has
// been assembled from its constituent components (body + neck + tuning pegs +
// bridge + strings + …).  The macroscopic B-rep of the stock is unchanged
// (the workpiece IS the assembled instrument blank); this skill stamps a
// signature recording instrument_type and component_count so QA, packaging,
// and serial-record steps can downstream verify the build.
//
//   ┌─────────┐   instrument_assemble   ┌─────────┐
//   │ blank   │  ───────────────────▶   │ blank   │  (shape identical;
//   │         │  (type, n components)   │ tagged  │   build metadata stamped)
//   └─────────┘                         └─────────┘
//
// Signature pattern:
//   - pattern.kind             = "instrument_assemble"
//   - pattern.instrument_type  = "guitar" | "violin" | "piano" | …
//   - pattern.component_count  = int (>= 2)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   instrument_type non-empty            (DFM-INPUT error if empty)
//   component_count >= 2                 (DFM-ASM-COUNT error if < 2)
//   component_count <= 500               (DFM-ASM-COUNT info if very high)
//
// Synthesis:  NO geometric change.  Clone shape + material, stamp signature.
// Recognize:  metadata-only replay.

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace instrument_assemble {

constexpr const char* kSkillId = "instrument_assemble";

struct Input
{
    std::string instrument_type = "guitar";
    int         component_count = 6;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace instrument_assemble
}  // namespace koocadcam::skill
