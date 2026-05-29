#pragma once
// @lat: [[engine/skills#blend_morph_two_shapes]]
//
// blend_morph_two_shapes — Compound morph skill.  Takes one cross-section
// from a "source A" pose (radius_a at z_a) and one from a "source B" pose
// (radius_b at z_b) and blends them with G¹ tangent continuity using
// BRepOffsetAPI_ThruSections.  The resulting solid is FUSED onto the
// workpiece (additive bridge), giving a smooth-blended transition that
// connects two coaxial cylindrical features.
//
// Pipeline (REAL multi-cut, ≥ 2 sub-features):
//   1. Build circular wire A at (centre_a, radius_a, z_a).
//   2. Build circular wire B at (centre_b, radius_b, z_b).
//   3. ThruSections({wireA, wireB}) with isSolid=true → blend solid.
//   4. prim::fuse(workpiece, blend_solid) → workpiece + bridge.
//
// DFM:
//   DFM-INPUT          : radius_a / radius_b > 0; z_a ≠ z_b.
//   DFM-BLEND-MIN-D    : the radii must be ≥ 0.5 mm (DFM-002 derivative;
//                        agent-chosen: 0.5 mm matches Tier-2 EDM-cut
//                        minimum from internal DFM rules).
//   DFM-BLEND-SLOPE    : (|radius_a − radius_b| / |z_a − z_b|) ≤ 5.0
//                        — the blend would otherwise have walls steeper
//                        than a 5:1 slope (manufacturability gate per
//                        DIN 8580 sheet-blend guidance, agent-chosen).

#include "Skill.hpp"

#include <gp_Pnt.hxx>

namespace koocadcam::skill {

class Workpiece;

namespace blend_morph_two_shapes {

constexpr const char* kSkillId = "blend_morph_two_shapes";

struct Input
{
    // Source section A
    gp_Pnt   centre_a   { 0.0, 0.0, 0.0 };
    double   radius_a   = 0.0;
    double   z_a        = 0.0;
    // Source section B
    gp_Pnt   centre_b   { 0.0, 0.0, 0.0 };
    double   radius_b   = 0.0;
    double   z_b        = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace blend_morph_two_shapes
}  // namespace koocadcam::skill
