#pragma once
// @lat: [[engine/skills#polish_op]]
//
// polish_op — mirror-finish polishing operation.  Removes a vanishingly thin
// uniform layer (~ 0.001 – 0.05 mm) from a planar face, restamping the face
// with a surface-finish (Ra) metadata target.
//
//        ┌─────────────┐               ┌─────────────┐
//        │             │               │             │
//        │             │   polish_op   │ (face Ra ≈  │
//        │             │  ────────▶   │  target_ra) │
//        │             │               │             │
//        └─────────────┘               └─────────────┘
//        (entry_face)                  (face slightly thinner,
//                                       mirror finish)
//
// Signature pattern:
//   - pattern.kind = "polish_op"
//   - pattern.target_ra = e.g. "ra_0.05"
//   - geometry CHANGES only by `removal_mm` (microns, near OCCT tolerance)
//
// DFM checks:
//   removal_mm ∈ [0.001, 0.05]
//   target_ra string non-empty
//
// Synthesis:
//   Skim a very thin layer (face_milling-style) along the face inward
//   normal.  If `removal_mm` < OCCT tolerance (1e-3 mm) the geometry is
//   left UNCHANGED — only the signature is restamped.
//
// Recognize:
//   Impossible from geometry alone (Ra is a sub-OCCT surface property).
//   Returns ONLY signatures replayed from `workpiece.features()` filtered
//   by this skill_id — confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace polish_op {

constexpr const char* kSkillId = "polish_op";

struct Input
{
    FaceDatum   entry_face;
    double      removal_mm = 0.005;          // 5 microns by default (mirror polish)
    std::string target_ra  = "ra_0.05";      // Ra in microns, encoded as string
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace polish_op
}  // namespace koocadcam::skill
