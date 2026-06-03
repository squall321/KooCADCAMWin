#pragma once
// @lat: [[engine/skills#variable_radius_fillet]]
//
// variable_radius_fillet — SolidWorks-style "Variable Size Fillet".
//
// Each selected edge gets a fillet whose radius varies linearly between
// radius_start_mm at the edge's first endpoint and radius_end_mm at the
// edge's last endpoint.  OCCT's BRepFilletAPI_MakeFillet supports this
// via the (r1, r2, edge) overload of Add().
//
// Edge selection is a Z-band: every edge whose bounding-box midpoint Z is
// within [edge_z_band_min, edge_z_band_max] is filleted.
//
// Signature pattern:
//   pattern.kind                  = "variable_radius_fillet"
//   pattern.is_compound           = true
//   pattern.edge_count_filleted   = #edges actually added (after Build skips)
//   pattern.radius_start_mm       = r_start
//   pattern.radius_end_mm         = r_end
//   pattern.derived_volume_removed= signed volume delta (negative)
//
// DFM:
//   DFM-INPUT     : both radii > 0, z_min < z_max.
//   DFM-WALL      : max(radius) must be < half the workpiece min-extent.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace variable_radius_fillet {

constexpr const char* kSkillId = "variable_radius_fillet";

struct Input
{
    double edge_z_band_min = 0.0;
    double edge_z_band_max = 0.0;
    double radius_start_mm = 1.0;
    double radius_end_mm   = 1.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace variable_radius_fillet
}  // namespace koocadcam::skill
