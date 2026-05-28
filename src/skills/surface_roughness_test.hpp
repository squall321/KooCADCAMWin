#pragma once
// @lat: [[engine/skills#surface_roughness_test]]
//
// surface_roughness_test — INSPECTION skill: contact stylus or non-
// contact optical profilometer Ra / Rz measurement at a named face.
// Geometrically a NO-OP.
//
//   diamond stylus  ───▶  face
//        │                ────╲╱╲╱──── ← micro-profile
//        │
//      trace
//        │   ──▶  Ra (μm) = arithmetic mean roughness
//        │       Rz (μm) = avg peak-to-valley over five sample lengths
//
// What we record:
//   - target face datum
//   - target_ra_um         — design intent (e.g. 0.8 µm for a bearing seat)
//   - measured_ra_um       — actual measured value (0 ⇒ planning only)
//   - cutoff_length_mm     — ISO 4288 lambda_c (typ. 0.8 mm)
//   - conformance bool     — measured ≤ target ⇒ true
//
// Input:
//   target_face_datum    — face to measure
//   target_ra_um         — design-intent Ra in µm
//   measured_ra_um       — measured Ra in µm; 0 ⇒ planning
//   cutoff_length_mm     — ISO 4288 cutoff lambda_c (default 0.8 mm)
//
// DFM checks:
//   target face datum resolves
//   target_ra_um > 0 (error if not)
//   target_ra_um < 0.01 ⇒ info  (sub-10nm Ra exotic — needs interferometer)
//   target_ra_um > 50  ⇒ info  (very rough — Ra rarely characterised)
//   cutoff_length_mm > 0 (error)
//   measured_ra_um < 0 (error — physical impossibility)
//   measured_ra_um > target_ra_um ⇒ warning (out-of-spec)
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace surface_roughness_test {

constexpr const char* kSkillId = "surface_roughness_test";

struct Input
{
    FaceDatum  target_face_datum;
    double     target_ra_um       = 1.6;
    double     measured_ra_um     = 0.0;    // 0 ⇒ planning only
    double     cutoff_length_mm   = 0.8;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace surface_roughness_test
}  // namespace koocadcam::skill
