#pragma once
// @lat: [[engine/skills#add_draft_to_face]]
//
// add_draft_to_face — Apply a real geometric draft angle to a (vertical) face
// using BRepOffsetAPI_DraftAngle.  The taper is built around a NEUTRAL plane
// (the `base_face`) whose normal defines the pull direction.
//
// SolidWorks "Draft Angle" body feature.
//
//          before               after (3° draft, pull = +Z)
//        ┌─────────┐                  ╲─────────╱
//        │  side   │   ───►            ╲ side  ╱
//        │  face   │                    ╲ face╱
//        └─────────┘                     ╲___╱
//
// Algorithm:
//   1. Resolve source & base faces.
//   2. Pull direction = base face outward normal.
//   3. drafter.Add(face, pullDir, angle_rad, neutralPlane).
//   4. drafter.Build().  If AddDone() == false OR Build() throws → emit
//      DFM-DRAFT-FAILED SkillError.
//
// Signature pattern:
//   pattern.kind             = "add_draft_to_face"
//   pattern.is_compound      = true
//   pattern.parameters       = { source_face_id, base_face_id, draft_angle_deg }
//   pattern.derived_volume   = volume_after - volume_before  (signed)
//
// DFM:
//   DFM-DRAFT-RANGE   : draft_angle_deg ∈ [0.5, 15].
//   DFM-DRAFT-FAILED  : OCCT draft engine could not build.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace add_draft_to_face {

constexpr const char* kSkillId = "add_draft_to_face";

struct Input
{
    int    source_face_id   = -1;      // face that gets tapered
    int    base_face_id     = -1;      // neutral plane (stays unchanged)
    double draft_angle_deg  = 1.0;     // expected ∈ [0.5, 15.0]
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace add_draft_to_face
}  // namespace koocadcam::skill
