#pragma once
// @lat: [[engine/skills#surface_trim_by_plane]]
//
// surface_trim_by_plane — Trim (cut) the workpiece with a half-space.
// SolidWorks "Trim Surface (Standard, by plane)" operation.
//
//   plane_normal ↑
//        │           ╔════════════════╗
//        │   keep    ║ workpiece      ║
//        ●────────── ╠════════════════╣  ← plane(origin, normal)
//        │           ║ discard        ║
//        │           ╚════════════════╝
//
// Algorithm:
//   1. Build a TopoDS_Face on the cutting plane.
//   2. BRepPrimAPI_MakeHalfSpace using a reference point on the DISCARD
//      side to materialize the half-space-to-subtract.
//   3. result = pr::cut(workpiece, halfspace_to_discard).
//
// Signature pattern:
//   pattern.kind             = "surface_trim_by_plane"
//   pattern.is_compound      = true
//   pattern.source_face_count= 1     (the cutting plane)
//   pattern.thickness_mm     = 0     (no offset — flat trim)
//   pattern.plane_origin / plane_normal / keep_side
//   pattern.derived_volume   = volume of the result body
//
// DFM:
//   DFM-INPUT     : workpiece must be non-null.
//   DFM-NORMAL    : plane_normal must be non-zero length.
//   DFM-KEEP-SIDE : keep_side must be "above" or "below".
//   DFM-MISS      : plane must intersect the workpiece bbox.

#include "Skill.hpp"

#include <array>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace surface_trim_by_plane {

constexpr const char* kSkillId = "surface_trim_by_plane";

struct Input
{
    std::array<double,3>  plane_origin = { 0.0, 0.0, 0.0 };
    std::array<double,3>  plane_normal = { 0.0, 0.0, 1.0 };
    // "above" keeps the +normal side; "below" keeps the -normal side.
    std::string           keep_side    = "above";
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace surface_trim_by_plane
}  // namespace koocadcam::skill
