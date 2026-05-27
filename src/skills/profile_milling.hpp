#pragma once
// @lat: [[engine/skills#profile_milling]]
//
// profile_milling — mill the OUTER PROFILE of a workpiece (silhouette edge).
// Removes a "shell" of material around the perimeter of the entry face,
// leaving an inset block.  For a cuboid stock this means a rectangular
// frame removed from the top down to `depth_mm`, leaving an island with
// outer extent = bbox_extent − 2·offset_in_mm.
//
//        Top view (looking down at entry_face):
//
//          ┌─────────────────────────────────┐  ← bbox boundary
//          │  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  ← outer shell to remove
//          │  ░  ┌───────────────────────┐ ░ │     (width = offset_in_mm)
//          │  ░  │                       │ ░ │
//          │  ░  │     ISLAND (kept)     │ ░ │
//          │  ░  │                       │ ░ │
//          │  ░  └───────────────────────┘ ░ │
//          │  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │
//          └─────────────────────────────────┘
//             ↑─── offset_in_mm ───↑
//
// Implementation strategy (the "annular box" cutter):
//   1. Compute bbox of the workpiece.
//   2. Build OUTER cutter box covering bbox + small overhang in entry-face
//      normal, depth = `depth_mm`.
//   3. Build INNER subtractor box matching the INSET footprint (bbox − 2·
//      offset), same depth.
//   4. The annular CUTTER = outer − inner.  Boolean cut: workpiece − cutter
//      produces the inset.
//
// Signature pattern (what the skill leaves on the workpiece):
//   - 4 new planar wall faces (the inset's vertical walls)
//   - top entry_face's outer edges are RECESSED (the original top corners
//     drop down by `depth_mm` — leaving the island at full height)
//   - 1 new annular planar face = the "step" at the bottom of the shell,
//     surrounding the island
//
// DFM checks:
//   offset_in_mm ≥ 0.4 mm  (DFM-001 analog: leave enough wall after profile)
//   depth_mm     > 0
//   bbox cross-section must accommodate the inset (offset < bbox_min_extent/2)
//
// Recognize:
//   Detect by comparing the workpiece's top-face outline to its bbox: if
//   the top entry_face is an inset rectangle smaller than bbox.dx × bbox.dy,
//   the offset can be recovered.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace profile_milling {

constexpr const char* kSkillId = "profile_milling";

struct Input
{
    FaceDatum   entry_face;                        // top planar face being profiled
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 }; // into the entry face
    double      offset_in_mm   = 1.0;              // shell thickness removed
    double      depth_mm       = 0.0;              // how far down from entry_face
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace profile_milling
}  // namespace koocadcam::skill
