#pragma once
// @lat: [[engine/skills#ream]]
//
// ream — PRECISION FINISH on an existing hole.  Slightly enlarges the
// diameter (typical 0.05–0.20 mm radial increase) for tight tolerance and
// smooth (mirror-like) surface finish.  Performed AFTER drill_hole, runs at
// slower feed with a multi-flute reamer tool.
//
//      Before:                          After:
//                                       (radius += enlarge_by_mm)
//                                       ┌─ entry_face
//                                       ▼
//          ┌─┬─┐                  ┌──┬───┬──┐
//          │ │ │                  │  │   │  │
//          │ │ │                  │  │   │  │
//          │ │ │       ─→         │  │   │  │
//          │ │ │                  │  │   │  │
//          └─┴─┘                  └──┴───┴──┘
//          d=R                   d=R+enlarge_by
//
// The output BORE LOOKS like a slightly larger drill_hole.  The DIFFERENCE
// is process-side: ream uses a reamer tool with characteristic feed/speed
// (slower feed, multi-flute, often with cutting oil), so the FeatureSignature
// records `pattern.kind = "ream"` + `enlarge_by_mm` so downstream CAM can
// distinguish from drill_hole.
//
// Input modes:
//   - explicit cylinder face: `FaceIdRef` of the existing bore
//   - inferred:               `FaceCylinderByAxis` (axis + tolerance)
//
// DFM checks:
//   enlarge_by_mm ∈ [0.02, 0.30]
//   the workpiece must contain a recognizable cylindrical face matching the
//   datum (else apply() throws).
//
// Recognize:
//   HARD alone.  A ream output is topologically identical to a drill_hole at
//   the larger diameter — without prior state we cannot tell "drilled to D"
//   from "drilled to D - δ then reamed by δ/2".  Therefore recognize()
//   returns LOW-CONFIDENCE candidates (≤ 0.50) only when a cylindrical bore
//   is in a typical reamed range, AND `drill_hole::recognize` would also
//   match.  Mostly useful for forward CAM planning: the process planner can
//   keep ream as an explicit history step but analysis primarily returns
//   drill_hole as the headline feature.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ream {

constexpr const char* kSkillId = "ream";

struct Input
{
    // EXISTING cylindrical bore on the workpiece.  Use FaceIdRef for an
    // exact face index, or FaceCylinderByAxis to find by axis line.
    FaceDatum   existing_hole_datum;

    // Radial increase (delta-radius).  Resulting bore radius = old + this.
    double      enlarge_by_mm = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ream
}  // namespace koocadcam::skill
