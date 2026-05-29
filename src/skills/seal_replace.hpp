#pragma once
// @lat: [[engine/skills#seal_replace]]
//
// seal_replace — maintenance record for replacing a static or dynamic
// elastomeric seal (O-ring, lip seal, gasket, mechanical face seal, …).
// Seals are sub-OCCT (compressed elastomer cross-sections are not modelled
// as B-rep); this skill captures the swap + the post-install leak test
// result so CMMS can close the work order.
//
//        ─────┬─────────────────────────┬─────
//             │                         │
//             │  ●●●●●●●●●●●●●●●●●●●  │   ← seal cross-section
//             │                         │
//        ─────┴─────────────────────────┴─────
//                       ↑
//             seal_type "O-ring", id "AS568-214",
//             leak_test_passed = true
//
// Signature pattern:
//   - pattern.kind              = "seal_replace"
//   - pattern.seal_type         = "O-ring" | "lip" | "gasket" | "mechanical" | ...
//   - pattern.seal_id           = catalog/PN string
//   - pattern.leak_test_passed  = bool (post-install Bubble / He / pressure-decay)
//   - pattern.geometry_changed  = false
//
// DFM checks:
//   seal_type non-empty                      (DFM-INPUT error)
//   seal_id   non-empty                      (DFM-INPUT error)
//   leak_test_passed == false                (DFM-SEAL-LEAK error — block apply)
//
// Synthesis:
//   NO geometric change — seal is a sub-OCCT elastomer.  Clone shape +
//   material verbatim and stamp the signature.
//
// Recognize:
//   Metadata-only replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace seal_replace {

constexpr const char* kSkillId = "seal_replace";

struct Input
{
    std::string seal_type        = "O-ring";
    std::string seal_id          = "AS568-214";
    bool        leak_test_passed = true;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace seal_replace
}  // namespace koocadcam::skill
