#pragma once
// @lat: [[engine/skills#glued_joint]]
//
// glued_joint — METADATA-ONLY adhesive bond between two named faces of the
// assembly.  Produces NO geometric change to the workpiece: the bond line is
// recorded purely as a FeatureSignature so the process planner / BOM
// generator can size adhesive bead, surface-prep, cure time, etc.
//
//   ─────────────────────────  ← face_a (this workpiece's bond face)
//   ░░░░░░░░░░░░░░░░░░░░░░░░░     adhesive layer (metadata, not B-rep)
//   ─────────────────────────  ← face_b (mating part's bond face)
//
// Signature pattern (what the skill leaves on the workpiece):
//   - shape unchanged (faceCount, edgeCount, vertexCount, volume all
//     identical pre/post apply)
//   - kind = "glued_joint" with the bond's face id, partner_part name,
//     adhesive type, bead_width / cure_time / shear strength metadata
//
// DFM checks:
//   DFM-GLUE-AREA      : bond face area ≥ 25 mm²    (too small → fragile)
//   DFM-GLUE-WIDTH     : bead_width_mm ∈ [0.1, 5.0] mm
//   DFM-GLUE-ADHESIVE  : adhesive_type non-empty
//   DFM-GLUE-CURE      : cure_time_min > 0
//
// Recognize:
//   ONLY by history replay (confidence 1.0).  There is no B-rep evidence of
//   a glued joint, so geometric fallback is impossible — recognize() returns
//   an empty vector when no signature is present.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace glued_joint {

constexpr const char* kSkillId = "glued_joint";

struct Input
{
    FaceDatum   bond_face;                       // face on THIS workpiece
    std::string partner_part      = "";          // name of the mating part
    std::string partner_face_name = "";          // name of the mating face on partner
    std::string adhesive_type     = "epoxy";     // "epoxy" | "cyanoacrylate" | "uv_cure" | …
    double      bead_width_mm     = 1.0;
    double      cure_time_min     = 60.0;
    double      shear_strength_MPa = 15.0;       // expected shear strength
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace glued_joint
}  // namespace koocadcam::skill
