#pragma once
// @lat: [[engine/skills#multi_radius_fillet_table]]
//
// multi_radius_fillet_table — SolidWorks "Multiple Radius Fillet": a list
// of (Z-band, radius) tuples, each applied to the edges in that band.
//
// Workflow:
//   for each edge in the workpiece
//     for each spec in edge_specs
//       if mid-Z(edge) ∈ [spec.z_min, spec.z_max]
//         BRepFilletAPI_MakeFillet.Add(spec.radius, edge)
//         break (first-match wins so an edge isn't double-added)

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace multi_radius_fillet_table {

constexpr const char* kSkillId = "multi_radius_fillet_table";

struct EdgeSpec
{
    double z_min     = 0.0;
    double z_max     = 0.0;
    double radius_mm = 1.0;
};

struct Input
{
    std::vector<EdgeSpec> edge_specs;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace multi_radius_fillet_table
}  // namespace koocadcam::skill
