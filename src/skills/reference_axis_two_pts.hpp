#pragma once
// @lat: [[engine/skills#reference_axis_two_pts]]
//
// reference_axis_two_pts — Construct a REFERENCE AXIS through two distinct
// points.  SolidWorks "Axis → Two Points / Vertices" datum.
//
// axis_origin = point_a
// axis_dir    = unit(point_b - point_a)
// axis_length = |point_b - point_a|
//
// Metadata-only: workpiece B-Rep unchanged.
//
// Signature pattern:
//   pattern.kind            = "reference_axis_two_pts"
//   pattern.is_reference    = true
//   pattern.datum_class     = "axis"
//   pattern.named_id        = <input.name>
//   pattern.point_a_xyz     = [ax, ay, az]
//   pattern.point_b_xyz     = [bx, by, bz]
//   pattern.axis_origin_xyz = [ax, ay, az]  (= point_a)
//   pattern.axis_dir_xyz    = [dx, dy, dz]  (unit)
//   pattern.axis_length     = <double>
//   pattern.geometry_changed= false
//
// DFM:
//   DFM-INPUT       : name empty.
//   DFM-REF-COINCIDENT : distance |B - A| < 1e-3 (zero-length axis).

#include "Skill.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace reference_axis_two_pts {

constexpr const char* kSkillId = "reference_axis_two_pts";

struct Input
{
    std::array<double,3> point_a = { 0.0, 0.0, 0.0 };
    std::array<double,3> point_b = { 0.0, 0.0, 1.0 };
    std::string          name    = "DATUM_AXIS_2PT";
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace reference_axis_two_pts
}  // namespace koocadcam::skill
