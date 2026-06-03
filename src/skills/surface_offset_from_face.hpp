#pragma once
// @lat: [[engine/skills#surface_offset_from_face]]
//
// surface_offset_from_face — Create a single Face that is the source face's
// outer wire translated by `offset_mm` along the source face normal.
// Distinct from `offset_surface` (which uses BRepOffsetAPI on the surface):
// this skill builds a planar twin from the outer wire only, which is more
// robust on non-developable input faces.
//
//             source face (plane / planar wire)
//            ╔═════════════════╗
//            ║                 ║
//            ╚═════════════════╝
//                  │ offset_mm  (along source normal)
//                  ▼
//            ╔═════════════════╗
//            ║ offset face     ║
//            ╚═════════════════╝
//
// Algorithm:
//   1. Resolve source_face → face & outward normal.
//   2. Compute translated origin = face_center + offset_mm * normal.
//   3. Build a plane at that origin parallel to the source plane.
//   4. Construct the offset Face from the source's outer wire transformed
//      to the new plane (BRepBuilderAPI_MakeFace).
//   5. Workpiece shape unchanged; offset Face attached as compound child.
//
// Signature pattern:
//   pattern.kind             = "surface_offset_from_face"
//   pattern.is_compound      = true
//   pattern.source_face_count= 1
//   pattern.offset_mm        = signed offset
//   pattern.derived_area     = area of the offset face
//
// DFM:
//   DFM-INPUT   : source face datum must resolve.
//   DFM-OFFSET  : offset_mm must be non-zero.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace surface_offset_from_face {

constexpr const char* kSkillId = "surface_offset_from_face";

struct Input
{
    FaceDatum  source_face;
    double     offset_mm = 1.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace surface_offset_from_face
}  // namespace koocadcam::skill
