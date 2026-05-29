#pragma once
// @lat: [[engine/skills#tear_test]]
//
// tear_test — destructive trouser-leg tear test (ASTM D624 / ISO 34-1)
// record for elastomers, films, foils and thin metal sheets.  A pre-cut
// specimen is pulled until tear propagates; the recorded tear force and
// specimen thickness define tear strength.  CAD geometry does not change.
//
//           ▲ pull
//           │
//        ───┤
//           │
//      ─────┘── nick / pre-cut
//      ─────┐── nick / pre-cut
//           │
//        ───┤
//           │
//           ▼ pull
//
// Signature pattern:
//   - pattern.kind             = "tear_test"
//   - pattern.tear_force_n
//   - pattern.sample_thickness_mm
//   - pattern.tear_strength_n_mm  (force / thickness)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   tear_force_n        > 0                  (DFM-INPUT error)
//   sample_thickness_mm > 0                  (DFM-INPUT error)
//   sample_thickness_mm ∈ [0.05, 25]         (DFM-TEAR-THICKNESS info)
//   tear_force_n        ≤ 10000              (DFM-TEAR-FORCE info)

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tear_test {

constexpr const char* kSkillId = "tear_test";

struct Input
{
    double tear_force_n        = 50.0;       // N
    double sample_thickness_mm = 2.0;        // mm
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tear_test
}  // namespace koocadcam::skill
