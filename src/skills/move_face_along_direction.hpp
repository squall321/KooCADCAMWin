#pragma once
// @lat: [[engine/skills#move_face_along_direction]]
//
// move_face_along_direction — Translate one face of the body by an offset,
// approximated as add/remove a thin prism on that face region.
//
// SolidWorks "Move Face" operation. True face-move with surrounding faces
// following (BRepFeat_LocalOperation / DPrism) is fragile in OCCT for
// generic faces. We approximate as follows:
//   - distance > 0  (face moves OUTWARD along +normal): add a thin prism
//                   (box of the face bbox footprint) extruded along `direction`
//                   for `distance_mm` and fuse it onto the body.
//   - distance < 0  (face moves INWARD opposite normal): cut a thin prism
//                   of `|distance|` mm depth on the face.
//
// Limitation: only planar faces are supported (the approximation builds
// an axis-aligned prism using the face bbox footprint).  For curved faces
// the recognizer / DFM rejects the input.  This matches SolidWorks Move
// Face for offset operations on planar regions.
//
// Signature pattern:
//   - kind = "move_face_along_direction"
//   - is_compound = false (single solid)
//   - direction_xyz: unit direction the face was moved along
//   - distance_mm:   signed offset distance
//   - face_anchor:   center of the moved face (post-op)
//
// DFM:
//   DFM-INPUT  : direction must be non-zero
//   DFM-INPUT  : |distance_mm| must be > 1e-6
//   DFM-INPUT  : face_id must be in range and PLANAR

#include "Skill.hpp"

#include <array>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace move_face_along_direction {

constexpr const char* kSkillId = "move_face_along_direction";

struct Input
{
    int                  face_id        = 0;
    std::array<double,3> direction_xyz  = { 0.0, 0.0, 1.0 };
    double               distance_mm    = 1.0;     // signed
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace move_face_along_direction
}  // namespace koocadcam::skill
