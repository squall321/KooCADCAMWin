#pragma once
// @lat: [[engine/skills#sandblast_op]]
//
// sandblast_op — abrasive surface texturing.  Adds surface-roughness
// metadata to the named face without changing the workpiece geometry.
// (The macroscopic shape stays identical; only the micro-texture changes.)
//
//        ┌─────────────┐
//        │ ░░░░░░░░░░░ │   ← sandblasted face (Ra ↑, matte appearance)
//        │             │
//        │             │
//        └─────────────┘
//
// Signature pattern:
//   - pattern.kind            = "sandblast_op"
//   - pattern.target_face_id  = face that was blasted
//   - pattern.grit_size       = e.g. "120_grit"
//   - pattern.target_ra       = e.g. "ra_3.2"
//   - pattern.pressure_psi    = blast pressure
//   - geometry IDENTICAL to input
//
// DFM checks:
//   pressure_psi ∈ (0, 100]   (above ~ 100 psi cratering risk on Al alloys)
//   grit_size & target_ra non-empty
//
// Synthesis:
//   NO geometric change.  Returns a CLONE of the workpiece carrying a new
//   FeatureSignature with this skill_id + the targeted face_id.
//
// Recognize:
//   Impossible from geometry (surface texture is sub-OCCT).  Returns ONLY
//   signatures filtered from `workpiece.features()`, confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sandblast_op {

constexpr const char* kSkillId = "sandblast_op";

struct Input
{
    FaceDatum   entry_face;
    std::string grit_size    = "120_grit";   // "60_grit" | "120_grit" | "220_grit" …
    double      pressure_psi = 60.0;         // typ. 40 – 80 psi
    std::string target_ra    = "ra_3.2";     // resulting Ra in microns
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sandblast_op
}  // namespace koocadcam::skill
