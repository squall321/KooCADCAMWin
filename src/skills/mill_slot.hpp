#pragma once
// @lat: [[engine/skills#mill_slot]]
//
// mill_slot — linear slot: rectangular pocket with two semicircular ends,
// the natural footprint of an end-mill traversing a straight line.
//
//                  entry_face (planar)
//                         │
//        ─────────────────┼──────────────────────
//                         │
//                  ╭──────┴──────────────╮    ↑ depth_mm
//                  │                     │    │   (flat bottom)
//                  ╰─────────────────────╯    ↓
//                  ↑                     ↑
//               start                   end
//
// Footprint (top view): stadium = box + 2 semicircles of radius width/2.
//
// Signature pattern:
//   - 2 cylindrical (half-cylinder) faces — the rounded ends
//   - 2 planar long-wall faces parallel to the start→end vector
//   - 1 planar flat-bottom face
//   - top: 2 half-circle edges + 2 straight edges
//   - bottom: same loop at z = bottom
//
// DFM checks:
//   DFM-002 : width_mm ≥ 0.8 mm  (slot width = tool diameter, end-mill min)
//   depth/width > 5 → warning
//
// Recognize:
//   1. Find pairs of cylindrical faces with matching radius and axes both
//      perpendicular to a common bottom planar face.
//   2. Verify 2 connecting planar walls exist, with normals perpendicular
//      to the line from cyl A axis to cyl B axis.
//   3. Recover: start = cyl_A center on entry plane, end = cyl_B center,
//      width = 2 × radius, depth = wall height.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mill_slot {

constexpr const char* kSkillId = "mill_slot";

struct Input
{
    FaceDatum   entry_face;
    double      start_x_mm = 0.0;
    double      start_y_mm = 0.0;
    double      end_x_mm   = 0.0;
    double      end_y_mm   = 0.0;
    gp_Dir      axis_dir   { 0.0, 0.0, -1.0 };
    double      width_mm   = 0.0;   // == tool diameter
    double      depth_mm   = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mill_slot
}  // namespace koocadcam::skill
