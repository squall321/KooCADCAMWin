#pragma once
// @lat: [[engine/skills#compression_test]]
//
// compression_test — destructive uniaxial compression test record (ASTM
// E9 / ISO 604 family).  A specimen or component is loaded between two
// platens at a controlled crosshead rate until either a target peak load
// or a target displacement is reached.  The CAD GEOMETRY does not change
// (this is a test record); the skill stamps the test parameters onto
// the workpiece so reliability engineering can replay the spec.
//
//            ▼ crosshead  (rate_mm_min)
//        ━━━━━━━━━━━━━━━
//             │ │ │ │
//             │ unit│        ← peak_load_kn at displacement_mm
//             │ │ │ │
//        ━━━━━━━━━━━━━━━
//                ▲ load cell
//
// Signature pattern:
//   - pattern.kind             = "compression_test"
//   - pattern.peak_load_kn
//   - pattern.displacement_mm
//   - pattern.rate_mm_min
//   - pattern.compressive_stress_proxy_mpa  (kN / nominal area lookup)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   peak_load_kn   > 0                  (DFM-INPUT error)
//   displacement_mm > 0                 (DFM-INPUT error)
//   rate_mm_min    > 0                  (DFM-INPUT error)
//   rate_mm_min    ∈ [0.001, 500]       (DFM-COMP-RATE info)
//   peak_load_kn   ≤ 1000               (DFM-COMP-LOAD info)

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace compression_test {

constexpr const char* kSkillId = "compression_test";

struct Input
{
    double peak_load_kn    = 10.0;        // kN
    double displacement_mm = 2.0;         // mm
    double rate_mm_min     = 5.0;         // mm/min crosshead
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace compression_test
}  // namespace koocadcam::skill
