#pragma once
// @lat: [[engine/skills#engrave_path]]
//
// engrave_path — engrave along an arbitrary 2D polyline on a face.  Used
// for logo outlines, alignment grooves, decorative channels.
//
//                   entry_face (planar)
//                         │
//        ─────────────────┼──────────────────────
//              ●─●─●─●    │             <- waypoints + connecting channel
//                  \      │
//                   ●─●   │
//                         │
//
// Signature pattern:
//   - In the FULL implementation: a wire that follows the waypoint polyline
//     extruded by depth_mm with the channel width applied as an offset.
//   - In this APPROXIMATION (slice 2): a chain of cylindrical spot cuts
//     (one per waypoint), each of diameter=width_mm and depth=depth_mm.
//     Connecting segments between waypoints are approximated by a series
//     of overlapping spot cuts along the segment (sampled at width_mm/2
//     stride) — produces a connected channel without invoking full
//     mill_slot logic.
//
// DFM checks:
//   DFM-019 : width_mm ≥ 0.15 mm
//   depth_mm ∈ [0.05, 0.5]
//   waypoints.size() ≥ 2
//
// Recognize:
//   Hard without metadata.  Heuristic: connected shallow cylindrical
//   "channels".  Falls back to metadata replay; if no signature is in
//   workpiece.features(), returns empty.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace engrave_path {

constexpr const char* kSkillId = "engrave_path";

struct Input
{
    FaceDatum                              entry_face;
    std::vector<std::array<double, 2>>     waypoints;     // (x_mm, y_mm) pairs
    double                                 width_mm  = 0.2;  // tool diameter
    double                                 depth_mm  = 0.15;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace engrave_path
}  // namespace koocadcam::skill
