#pragma once
// @lat: [[engine/skills#reference_plane_at_3pts]]
//
// reference_plane_at_3pts — Construct a REFERENCE PLANE through three
// non-collinear points.  SolidWorks "Plane → 3 Points" datum.
//
// Plane normal:   n = unit( (B - A) x (C - A) )
// Plane center:   centroid = (A + B + C) / 3
//
// Metadata-only: workpiece B-Rep is unchanged; the plane is captured in the
// FeatureSignature so downstream skills can consume it by `named_id`.
//
// Signature pattern:
//   pattern.kind            = "reference_plane_at_3pts"
//   pattern.is_reference    = true
//   pattern.datum_class     = "plane"
//   pattern.named_id        = <input.name>
//   pattern.point_a_xyz     = [ax, ay, az]
//   pattern.point_b_xyz     = [bx, by, bz]
//   pattern.point_c_xyz     = [cx, cy, cz]
//   pattern.plane_origin_xyz= [centroid]
//   pattern.plane_normal_xyz= [nx, ny, nz]   (unit, may be flipped sign-wise)
//   pattern.geometry_changed= false
//
// DFM:
//   DFM-INPUT          : name empty.
//   DFM-REF-COLLINEAR  : cross-product magnitude < 1e-3 (collinear points,
//                        plane is undefined).

#include "Skill.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace reference_plane_at_3pts {

constexpr const char* kSkillId = "reference_plane_at_3pts";

struct Input
{
    std::array<double,3> point_a = { 0.0, 0.0, 0.0 };
    std::array<double,3> point_b = { 1.0, 0.0, 0.0 };
    std::array<double,3> point_c = { 0.0, 1.0, 0.0 };
    std::string          name    = "DATUM_PLANE_3PT";
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace reference_plane_at_3pts
}  // namespace koocadcam::skill
