#pragma once
// @lat: [[engine/skills#hardness_test]]
//
// hardness_test — INSPECTION skill: surface hardness indentation
// measurement at a single point on a chosen face.  Geometrically a
// NO-OP (the tiny indentation < 0.5 mm is not modelled in B-rep).
//
// **Design choice — single-point, not face-average.**
//
//   Rockwell, Vickers, and Brinell are inherently point-load indentation
//   tests: one indenter, one load, one impression per test cycle.  An
//   "average over a face" is achieved by chaining multiple hardness_test
//   features (each with its own location) in the process plan, NOT by
//   embedding a multi-point loop inside one skill.  This keeps the
//   skill's contract one-test-per-call (consistent with probe_point) and
//   leaves multi-point campaigns to scan_pattern-style orchestration.
//
//       diamond / carbide indenter
//                ▼
//          ╲   ╱
//           ●          ← single indent (< 0.5 mm impression)
//      ─────────────── ← target face
//
// What we record:
//   - target face datum
//   - scale                 — "HRC" | "HRB" | "HV" | "HB"
//   - indenter_type         — "diamond_brale" | "diamond_pyramid" |
//                             "tungsten_carbide_ball" | "steel_ball"
//   - load_kgf              — applied load in kgf (typical: HRC=150, HRB=100,
//                             HV=0.001-100, HB=3000)
//   - measured_hardness     — measured value in the chosen scale's units
//                             (0 ⇒ planning only)
//
// DFM checks:
//   target face datum resolves
//   scale is one of the four allowed
//   indenter_type is one of the four allowed
//   load_kgf > 0 (error)
//   measured_hardness ≥ 0 (error if negative)
//   scale ↔ indenter compatibility (info if mismatched, e.g. HRC needs
//   diamond_brale, HV needs diamond_pyramid, HB needs ball)
//   load_kgf outside typical range for scale → info
//
// Recognize:
//   Metadata-only — no geometric fingerprint at this scale.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hardness_test {

constexpr const char* kSkillId = "hardness_test";

struct Input
{
    FaceDatum     target_face_datum;
    std::string   scale                = "HRC";              // HRC | HRB | HV | HB
    std::string   indenter_type        = "diamond_brale";    // see header
    double        load_kgf             = 150.0;
    double        measured_hardness    = 0.0;                // 0 ⇒ planning
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hardness_test
}  // namespace koocadcam::skill
