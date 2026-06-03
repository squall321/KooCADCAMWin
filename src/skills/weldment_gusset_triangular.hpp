#pragma once
// @lat: [[engine/skills#weldment_gusset_triangular]]
//
// weldment_gusset_triangular — Triangular gusset plate fillet between two
// intersecting structural members.  Computes a triangle with corner at the
// meeting point and equal legs along the two member axes, builds the
// triangular face, extrudes by `gusset_thickness_mm` perpendicular to the
// triangle plane, FUSES onto the workpiece.
//
//          vertex ──── leg_a (unit·gusset_leg_length)
//             \\
//              \\
//               leg_b
//
// Signature pattern:
//   pattern.kind             = "weldment_gusset_triangular"
//   pattern.is_compound      = true
//   pattern.profile_kind     = "triangle"
//   pattern.derived_volume_mm3
//
// DFM:
//   DFM-INPUT      : thickness > 0, leg_length > 0, leg vectors non-zero.
//   DFM-GEOM       : legs must be non-collinear (need a real triangle).

#include "Skill.hpp"

#include <array>

namespace koocadcam::skill {

class Workpiece;

namespace weldment_gusset_triangular {

constexpr const char* kSkillId = "weldment_gusset_triangular";

struct Input
{
    std::array<double, 3>  vertex_xyz            = { 0.0, 0.0, 0.0 };
    std::array<double, 3>  leg_a_xyz             = { 1.0, 0.0, 0.0 };
    std::array<double, 3>  leg_b_xyz             = { 0.0, 1.0, 0.0 };
    double                 gusset_thickness_mm   = 6.0;
    double                 gusset_leg_length_mm  = 80.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace weldment_gusset_triangular
}  // namespace koocadcam::skill
