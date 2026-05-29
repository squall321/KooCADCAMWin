#pragma once
// @lat: [[engine/skills#shape_morph_interpolate]]
//
// shape_morph_interpolate — Compound shape-morph skill.  Builds a solid
// that is the geometric interpolation between two closed cross-section
// wires A and B at parameter t ∈ [0, 1] (t=0 → shape A, t=1 → shape B),
// then SUBTRACTS that interpolated solid from the workpiece.
//
// Pipeline (REAL multi-cut, ≥ 2 sub-features):
//   1. Build the intermediate section S(t) by lerping A and B vertex-wise.
//   2. Build N − 2 additional intermediate sections at sample parameters
//      around t to give ThruSections enough waypoints for a smooth blend.
//   3. BRepOffsetAPI_ThruSections through the N wires → morph solid.
//   4. prim::cut(workpiece, morph_solid) → final cavity.
//
// This is genuinely compound: ThruSections itself fuses N section wires,
// and the final Boolean cut is a second geometric op.  subfeature_count
// is recorded as (N_sections + 1 cut) ≥ 3 in practice.
//
// DFM:
//   DFM-INPUT          : section_a / section_b must each have ≥ 3 distinct
//                        vertices.  t must lie in [0, 1].  Both sections
//                        must have matching vertex counts.
//   DFM-MORPH-MIN-FT   : final cavity floor thickness ≥ 0.5 mm
//                        (agent-chosen, motivated by min EDM wall guidance
//                        in DIN 8580 manufacturing-class rules).

#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace shape_morph_interpolate {

constexpr const char* kSkillId = "shape_morph_interpolate";

struct Input
{
    // section_a / section_b: closed polylines (each ≥ 3 distinct vertices,
    // same vertex count so a vertex-wise lerp is well-defined).
    std::vector<gp_Pnt>  section_a;
    std::vector<gp_Pnt>  section_b;
    // Interpolation parameter (0 = pure A, 1 = pure B).
    double               t = 0.5;
    // Number of intermediate ThruSection waypoints (≥ 2; minimum useful: 3).
    int                  intermediate_sections = 3;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace shape_morph_interpolate
}  // namespace koocadcam::skill
