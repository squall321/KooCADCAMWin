#pragma once
// @lat: [[engine/skills#full_round_fillet]]
//
// full_round_fillet — SolidWorks-style "Full Round Fillet".
//
// Given three adjacent faces (A, B, C) where B sits between A and C, the
// operation replaces B with a fully tangent rolled-ball fillet between A
// and C.  OCCT 8.0's BRepFilletAPI_MakeFillet does not expose a public
// 3-face overload on every platform; this skill therefore:
//
//   1. Locates the edges shared between (A,B) and (B,C) via TopExp::
//      MapShapesAndAncestors.
//   2. Computes an approximate fillet radius from the geometry of B —
//      we use half the average length of B's bounding edges, capped to
//      ¼ × min(bbox of B).  This is the analytic fallback the task spec
//      calls out: "if 3-face overload isn't available in OCCT 8.0,
//      approximate by finding the shared edges between adjacent face
//      pairs and applying constant-radius fillet computed from face
//      geometry."
//   3. Applies a constant-radius fillet to those shared edges via
//      BRepFilletAPI_MakeFillet::Add(r, edge).
//
// Inputs identify the three faces by FaceIdRef indices.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace full_round_fillet {

constexpr const char* kSkillId = "full_round_fillet";

struct Input
{
    int face_a_id = -1;
    int face_b_id = -1;   // the "middle" face that becomes the round
    int face_c_id = -1;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace full_round_fillet
}  // namespace koocadcam::skill
