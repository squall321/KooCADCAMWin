#pragma once
// @lat: [[engine/skills#draft_angle_apply]]
//
// draft_angle_apply — flags a (nominally vertical) face on the workpiece as
// requiring a draft taper between 1° and 5° for mould release.  Slice-1
// implementation is METADATA-ONLY: the skill validates that the target face
// exists and is roughly vertical, then records (face_id, draft_deg) in the
// FeatureSignature so downstream CAM / process-plan tools can apply or
// verify the actual draft in a follow-on operation.
//
// Rationale for metadata-only (vs. real face replacement):
//   * Reliable geometric draft on an arbitrary planar face requires either
//     `BRepOffset_MakeOffset` (which is fragile on non-developable joins) or
//     a custom face-replacement loft that re-stitches all adjacent faces.
//     For slice-1 this is high risk and out of scope.
//   * The metadata-only path still satisfies the 7-condition skill contract:
//     apply records a signature, validate enforces the draft range and the
//     verticality requirement, recognize replays the recorded face_id.
//
// Behavior:
//   * apply: runs validate; if ok, builds a new Workpiece with the SAME
//     shape and a FeatureSignature recording { face_id, draft_deg }.
//   * Per the prompt: "If too complex, use metadata-only with face_id +
//     draft_deg." → we do exactly that.
//
// Signature pattern:
//   pattern.kind             = "draft_angle_apply"
//   pattern.face_id          = resolved face index
//   pattern.draft_deg        = recorded angle
//   pattern.applied_geometry = false   (metadata-only marker)
//
// DFM:
//   DFM-DRAFT-RANGE   : draft_deg ∈ [1.0, 5.0]
//   DFM-DRAFT-VERT    : entry face must be (approximately) vertical
//                       (|n·Z| < 0.2)

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace draft_angle_apply {

constexpr const char* kSkillId = "draft_angle_apply";

struct Input
{
    FaceDatum   target_face;
    double      draft_deg = 0.0;       // expected ∈ [1.0, 5.0]
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace draft_angle_apply
}  // namespace koocadcam::skill
