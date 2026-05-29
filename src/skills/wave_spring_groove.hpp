#pragma once
// @lat: [[engine/skills#wave_spring_groove]]
//
// wave_spring_groove — COMPOUND FEATURE (2 primitive sub-cuts on a bore).
//
// Cylindrical-bore retention groove for a Smalley / Rotor Clip wave spring.
// Cut on the inside wall of an existing bore.  Per Smalley catalog C600:
//
//   Sub-feature chain:
//     1. Main slot groove   — annular cut (cylinder of radius mean_dia/2)
//                             into the bore wall, width = wave spring height
//                             × 1.05 (5% running clearance).
//     2. Bottom relief      — small undercut at one corner of the groove
//                             for chip evacuation (per Smalley fitting
//                             guide §4.2), radial extension = 0.15 × width.
//
//   Both cutters share the same bore axis; the groove cutter is a thick
//   annular band (outer = mean_dia/2 + relief, inner = mean_dia/2 −
//   relief / 2).  Fused once, then a single cut from the workpiece.
//
// Output topology: 1 cylindrical face (groove root) + 2 planar side walls
// + 1 small relief annulus.  Single Workpiece, single FeatureSignature
// with pattern.is_compound = true, subfeature_count = 2.
//
// DFM gates (Smalley C600 wave-spring retention groove):
//   - mean_dia >= 5 mm and <= 250 mm (covered catalog range)
//   - wave_spring_id < mean_dia (must fit IN the bore wall — inner-cyl
//     retention)
//   - groove width = wave_spring_height × 1.05 must be >= 0.5 mm
//   - groove depth = (mean_dia − wave_spring_id) / 2 must be in
//     [0.2 mm, mean_dia × 0.05] (Smalley range; too deep weakens wall)
//
// Recognize: metadata replay (compound groove leaves the same topological
// pattern as many bore features; uniqueness comes from history).
// Returns confidence 1.0 when matched.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wave_spring_groove {

constexpr const char* kSkillId = "wave_spring_groove";

struct Input
{
    gp_Ax1   bore_axis        { gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0) };
    double   mean_dia         = 0.0;    // OD of groove cut, mm
    double   wave_spring_id   = 0.0;    // wave spring inside diameter, mm
    double   wave_spring_height = 0.0;  // axial height of the spring (un-loaded), mm
    double   axial_position_mm = 0.0;   // distance from bore_axis.Location() along axis
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wave_spring_groove
}  // namespace koocadcam::skill
