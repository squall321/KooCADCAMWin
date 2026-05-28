#pragma once
// @lat: [[engine/skills#blend_surface]]
//
// blend_surface — fill the gap between two existing faces with a smooth
// surface, producing an additive transition patch.  Conceptually a
// boundary-fill ("n-sided patch"): the boundary curves are the outer
// edges of `face_a` and `face_b`, plus any optional rail/guide edges
// supplied; OCCT's BRepFill_Filling builds a face that interpolates
// those constraints with C¹ continuity.
//
//          ╔═══════════╗
//          ║ face_a    ║
//          ╚════•══════╝
//               │  ◄── blend (constructed)
//          ╔════•══════╗
//          ║ face_b    ║
//          ╚═══════════╝
//
// The result is fused onto the workpiece so that the resulting topology
// gains one extra face (the blend) plus the boundary edges connecting it
// to face_a / face_b.
//
// Signature pattern:
//   pattern.kind             = "blend_surface"
//   pattern.face_a_id        = resolved face index
//   pattern.face_b_id        = resolved face index
//   pattern.continuity       = "G1"
//
// DFM:
//   DFM-INPUT       : face datums must resolve.
//   DFM-BLEND-SAME  : face_a and face_b must be distinct.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace blend_surface {

constexpr const char* kSkillId = "blend_surface";

struct Input
{
    FaceDatum  face_a;
    FaceDatum  face_b;
    // 0 = positional (G0), 1 = tangent (G1).  Slice-1 always builds G1.
    int        continuity = 1;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace blend_surface
}  // namespace koocadcam::skill
