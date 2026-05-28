#pragma once
// @lat: [[engine/skills#offset_surface]]
//
// offset_surface — produce a parallel surface a constant distance away
// from a target face, using BRepOffsetAPI_MakeOffsetShape.  The new
// surface is fused onto the workpiece so the result contains both the
// original face and its offset twin (and a thin shell connecting them).
//
//         ╔══════════════════════╗
//         ║ original target face ║
//         ╚══════════════════════╝
//                  │ offset_mm
//                  ▼
//         ╔══════════════════════╗
//         ║ offset surface       ║
//         ╚══════════════════════╝
//
// Signature pattern:
//   pattern.kind         = "offset_surface"
//   pattern.target_face_id
//   pattern.offset_mm
//
// DFM:
//   DFM-INPUT         : offset_mm must be non-zero (sign decides direction).
//   DFM-OFFSET-MAG    : |offset_mm| < 0.5 × workpiece min-extent (else the
//                       offset will collide with the opposite side of the
//                       part and self-intersect).

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace offset_surface {

constexpr const char* kSkillId = "offset_surface";

struct Input
{
    FaceDatum   target_face;
    double      offset_mm  = 1.0;       // signed; positive = along face normal
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace offset_surface
}  // namespace koocadcam::skill
