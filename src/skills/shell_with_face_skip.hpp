#pragma once
// @lat: [[engine/skills#shell_with_face_skip]]
//
// shell_with_face_skip — Hollow the body with a uniform wall thickness while
// leaving one or more chosen faces OPEN (they are removed before offsetting).
// This is the SolidWorks "Shell" command.
//
//         ┌───────────────────┐                ┌───────────────────┐
//         │                   │                │ ┌───────────────┐ │
//         │      solid        │   shell + skip │ │               │ │
//         │      block        │  ─────────►    │ │  open cavity  │ │
//         │                   │                │ │               │ │
//         └───────────────────┘                └─┴───────────────┴─┘
//                                                  (top face removed)
//
// Algorithm:
//   1. Collect TopoDS_Face's for each id in `open_face_ids` via wp.face(id).
//   2. Call BRepOffsetAPI_MakeThickSolid::MakeThickSolidByJoin with offset =
//      -wall_thickness_mm (inward).
//   3. Build() under try/catch — emit SkillError("DFM-SHELL-FAILED", ...) on
//      throw.
//
// Signature pattern:
//   pattern.kind             = "shell_with_face_skip"
//   pattern.is_compound      = true
//   pattern.parameters       = { wall_thickness_mm, open_face_ids }
//   pattern.derived_volume   = volume_before - volume_after  (material removed)
//
// DFM:
//   DFM-INPUT          : wall_thickness_mm must be > 0.
//   DFM-SHELL-FAILED   : MakeThickSolid build threw or not-done.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace shell_with_face_skip {

constexpr const char* kSkillId = "shell_with_face_skip";

struct Input
{
    double            wall_thickness_mm = 1.0;
    std::vector<int>  open_face_ids;        // faces removed before offsetting
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace shell_with_face_skip
}  // namespace koocadcam::skill
