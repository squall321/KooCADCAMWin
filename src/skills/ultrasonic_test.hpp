#pragma once
// @lat: [[engine/skills#ultrasonic_test]]
//
// ultrasonic_test (UT) — INSPECTION skill: pulse-echo ultrasonic non-
// destructive evaluation of a face / subsurface volume.  Geometrically
// a NO-OP.
//
//     piezo probe         ┌─── workpiece face
//        │  ▼              │
//        │  ▼              │           reflected echo
//        │  ▼ ─ ─ ─ ─ ─ ─ ─┼─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
//        │  ▼              │ ▲
//        │  ▼              │ │
//        │  ▼              │ │  defect  ◢◣
//        │  ▼              │ │           ╲
//        └──▼──────────────┴─┴────────────●─── back wall echo
//
// What we record:
//   - target face datum
//   - freq_mhz             — transducer centre frequency (2.25 / 5 / 10 …)
//   - probe_dia_mm         — transducer crystal diameter (typ. 6-25 mm)
//   - area_mm2             — scan coverage area on the face
//   - defect_class         — categorical: "none" | "minor" | "major" | "critical"
//
// DFM checks:
//   target face datum resolves
//   freq_mhz in (0, 100]
//   probe_dia_mm in (0, 100]
//   area_mm2 > 0 and area_mm2 ≤ face area + 5 %
//   defect_class is one of the four allowed strings
//
// Material gating:
//   Ultrasonic propagation through highly attenuative materials (very
//   coarse castings, foamed polymers) is unreliable — emit info finding
//   for non-metallic / cast materials.  All metallic wrought materials
//   are OK; UT is canonical for forgings.
//
// Recognize:
//   Metadata-only operation — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ultrasonic_test {

constexpr const char* kSkillId = "ultrasonic_test";

struct Input
{
    FaceDatum     target_face_datum;
    double        freq_mhz       = 5.0;
    double        probe_dia_mm   = 10.0;
    double        area_mm2       = 100.0;
    std::string   defect_class   = "none";   // "none" | "minor" | "major" | "critical"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ultrasonic_test
}  // namespace koocadcam::skill
