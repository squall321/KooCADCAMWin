#pragma once
// @lat: [[engine/skills#radiographic_test]]
//
// radiographic_test (RT) — INSPECTION skill: X-ray / gamma-ray
// volumetric non-destructive evaluation.  X-ray source on one side of
// the part exposes a film or digital detector on the opposite side;
// internal porosity, inclusions, and lack-of-fusion appear as density
// differences in the image.  Geometrically a NO-OP.
//
//       X-ray tube                     workpiece                 film
//           ●                          ┌──────┐               ▒▓█▒▓█
//          ╱│╲           ─▶            │      │    ─▶         ░░██░░  ← void shows dark
//          ╲│╱                         │  ◯   │               ▒▓█▒▓█
//           ▼                          └──────┘
//
// What we record:
//   - target face datum
//   - source_kv            — X-ray tube voltage (50 - 450 kV typical)
//   - exposure_min         — exposure duration (0.5 - 30 min typical)
//   - film_class           — ISO 11699-1 film system class
//                            "C1" | "C2" | "C3" | "C4" | "C5" | "C6"
//
// DFM checks:
//   target face datum resolves
//   source_kv > 0 (error)
//   source_kv outside [40, 500] → info (rare regime)
//   exposure_min > 0 (error)
//   exposure_min > 60 → info (very long exposure)
//   film_class is a valid ISO 11699-1 class
//
// Material thickness/density note:
//   For thick / dense materials (steel > 25 mm at 100 kV) penetration is
//   marginal — emit info.  We treat material+thickness as combined hint.
//
// Recognize:
//   Metadata-only — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace radiographic_test {

constexpr const char* kSkillId = "radiographic_test";

struct Input
{
    FaceDatum     target_face_datum;
    double        source_kv      = 200.0;
    double        exposure_min   = 5.0;
    std::string   film_class     = "C3";    // ISO 11699-1: C1..C6
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace radiographic_test
}  // namespace koocadcam::skill
