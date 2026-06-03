#pragma once
// @lat: [[engine/skills#reference_point_on_face_uv]]
//
// reference_point_on_face_uv — Construct a REFERENCE POINT at parametric
// coordinates (u, v) on a face's surface.  SolidWorks "Point → On Surface
// (parametric)" datum.
//
// (u, v) are NORMALIZED into [0, 1] — they're linearly mapped to the face's
// underlying surface UV parameter range from BRepAdaptor_Surface.  This
// makes the reference robust against changes in the underlying surface's
// parameter conventions (lathe vs. extruded surfaces have different ranges).
//
// xyz_resolved = surface.Value(uMin + u*(uMax - uMin), vMin + v*(vMax - vMin))
//
// Metadata-only: workpiece B-Rep unchanged.
//
// Signature pattern:
//   pattern.kind            = "reference_point_on_face_uv"
//   pattern.is_reference    = true
//   pattern.datum_class     = "point"
//   pattern.named_id        = <input.name>
//   pattern.face_id         = <input.face_id>
//   pattern.u_norm          = <input.u in [0,1]>
//   pattern.v_norm          = <input.v in [0,1]>
//   pattern.point_xyz       = [x, y, z]   (BRepAdaptor_Surface evaluation)
//   pattern.geometry_changed= false
//
// DFM:
//   DFM-INPUT       : name empty.
//   DFM-INPUT       : face_id out of range.
//   DFM-INPUT       : u or v outside [0, 1] (rejected — caller must clamp).

#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace reference_point_on_face_uv {

constexpr const char* kSkillId = "reference_point_on_face_uv";

struct Input
{
    int          face_id = -1;
    double       u       = 0.5;     // normalized [0, 1]
    double       v       = 0.5;     // normalized [0, 1]
    std::string  name    = "DATUM_POINT_UV";
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace reference_point_on_face_uv
}  // namespace koocadcam::skill
