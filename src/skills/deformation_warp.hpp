#pragma once
// @lat: [[engine/skills#deformation_warp]]
//
// deformation_warp — Compound skill that warps an entire workpiece by
// composing TWO geometric transforms applied in sequence:
//   1. A non-uniform SCALE (sx, sy, sz) along world axes — applied via
//      BRepBuilderAPI_GTransform with a gp_GTrsf (the only OCCT transform
//      that accepts a non-uniform 3×3 matrix).
//   2. A RIGID rotation about an arbitrary axis through the origin —
//      applied via BRepBuilderAPI_Transform with a gp_Trsf.
//
// Each step is a separate, REAL geometry-modifying op.  The composite is
// recorded in the FeatureSignature so the warp can be inverted or replayed.
//
// DFM:
//   DFM-INPUT          : scale factors must be > 0.
//   DFM-WARP-LIMIT     : per-axis scale must lie in [0.1, 10.0] — beyond
//                        that the warp would change the part class.  Agent-
//                        chosen, motivated by ISO 286 dimensional-tolerance
//                        envelope (a 10× warp falls outside any practical
//                        manufacturing tolerance class).

#include "Skill.hpp"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

namespace koocadcam::skill {

class Workpiece;

namespace deformation_warp {

constexpr const char* kSkillId = "deformation_warp";

struct Input
{
    // Non-uniform scale per axis (≥ 1 expands, < 1 shrinks).
    double  scale_x = 1.0;
    double  scale_y = 1.0;
    double  scale_z = 1.0;
    // Rigid rotation about (axis_origin, axis_dir) by angle_deg.
    gp_Pnt  axis_origin { 0.0, 0.0, 0.0 };
    gp_Dir  axis_dir    { 0.0, 0.0, 1.0 };
    double  angle_deg   = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace deformation_warp
}  // namespace koocadcam::skill
