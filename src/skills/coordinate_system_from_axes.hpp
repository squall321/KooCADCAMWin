#pragma once
// @lat: [[engine/skills#coordinate_system_from_axes]]
//
// coordinate_system_from_axes — Construct a LOCAL COORDINATE SYSTEM (origin
// + X / Y / Z axes).  SolidWorks "Coordinate System" datum.
//
// Input: origin + X-axis direction + Y-axis direction.
// Computed:
//   x_unit = unit(x_axis_dir)
//   y_unit = unit(y_axis_dir)
//   z_unit = unit(x_unit x y_unit)
//
// Orthogonality is enforced to within 5° of the right angle between X & Y;
// otherwise validate() fails.  Z is always computed (X x Y) — even when X
// and Y are not perfectly orthogonal we record the user's raw y_axis_dir
// and the cross-product z_axis so downstream consumers can decide whether
// to Gram-Schmidt orthogonalise.
//
// Metadata-only: workpiece B-Rep unchanged.
//
// Signature pattern:
//   pattern.kind            = "coordinate_system_from_axes"
//   pattern.is_reference    = true
//   pattern.datum_class     = "coord_sys"
//   pattern.named_id        = <input.name>
//   pattern.origin_xyz      = [ox, oy, oz]
//   pattern.x_axis_xyz      = [xx, xy, xz]   (unit)
//   pattern.y_axis_xyz      = [yx, yy, yz]   (unit)
//   pattern.z_axis_xyz      = [zx, zy, zz]   (unit = X x Y)
//   pattern.angle_xy_deg    = <double>       (angle between X & Y)
//   pattern.geometry_changed= false
//
// DFM:
//   DFM-INPUT             : name empty.
//   DFM-INPUT             : x_axis_dir or y_axis_dir is zero-length.
//   DFM-REF-ORTHOGONALITY : angle between X & Y deviates > 5° from 90°.

#include "Skill.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace coordinate_system_from_axes {

constexpr const char* kSkillId = "coordinate_system_from_axes";

struct Input
{
    std::array<double,3> origin     = { 0.0, 0.0, 0.0 };
    std::array<double,3> x_axis_dir = { 1.0, 0.0, 0.0 };
    std::array<double,3> y_axis_dir = { 0.0, 1.0, 0.0 };
    std::string          name       = "CSYS_LOCAL";
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace coordinate_system_from_axes
}  // namespace koocadcam::skill
