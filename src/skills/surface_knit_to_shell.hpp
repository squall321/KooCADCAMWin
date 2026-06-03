#pragma once
// @lat: [[engine/skills#surface_knit_to_shell]]
//
// surface_knit_to_shell — Sew ≥ 3 selected faces of the workpiece into a
// single TopoDS_Shell using BRepBuilderAPI_Sewing.  SolidWorks "Knit
// Surface" operation.
//
//        face_a ─┐
//        face_b ─┼─► BRepBuilderAPI_Sewing.Add(...) → Shell
//        face_c ─┘
//
// The original solid is kept as the primary workpiece shape; the sewn
// Shell is added as a sub-step in a compound so downstream skills can
// access either form via TopExp_Explorer.
//
// Signature pattern:
//   pattern.kind             = "surface_knit_to_shell"
//   pattern.is_compound      = true
//   pattern.source_face_count= N (≥ 3)
//   pattern.derived_area     = total area of sewn faces
//
// DFM:
//   DFM-INPUT     : every face datum must resolve.
//   DFM-FACE-COUNT: at least 3 distinct faces required.
//   DFM-TOLERANCE : tolerance_mm must be > 0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace surface_knit_to_shell {

constexpr const char* kSkillId = "surface_knit_to_shell";

struct Input
{
    // Explicit face indices into the workpiece enumeration.
    std::vector<int>  face_ids;
    double            tolerance_mm = 1e-3;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace surface_knit_to_shell
}  // namespace koocadcam::skill
