#pragma once
// @lat: [[engine/skills#tube_hydroform]]
//
// tube_hydroform — internal-pressure expansion of a tube against a die so
// the outer wall conforms to a target axial profile (z, outer_R).  The
// classic automotive application is a structural frame member with multiple
// section changes along its length.
//
// METADATA-ONLY: the target profile can in principle be turned into a sweep,
// but doing so robustly for arbitrary polylines requires lofting (and may
// produce a non-manifold for sharp R steps).  We instead record the polyline
// in the signature; downstream solvers (CAE wall-thinning) read the profile
// from there.
//
//                  ┌─────┐       ┌─────────┐
//                  │     └───────┘         │     ← bulged-out die-matched
//                  │ z₁,R₁  z₂,R₂  z₃,R₃   │       profile (input polyline)
//                  └───────────────────────┘
//                  ◄──── tube stock ────►
//
// Signature pattern:
//   - pattern.kind                       = "tube_hydroform"
//   - pattern.target_profile_polyline    = [[z, outer_R], ...]
//   - pattern.pressure_mpa               = <input>
//   - pattern.geometry_changed           = false
//
// DFM checks:
//   DFM-INPUT          : polyline has ≥ 2 vertices, monotone Z, all R > 0
//   DFM-TUBE-HF-RATIO  : max(R) / min(R) ≤ 1.6 (typical practical expansion
//                        limit for single-stage tube hydroform)
//   DFM-TUBE-HF-PRESS  : pressure_mpa ∈ [50, 600]
//   DFM-TUBE-HF-Z      : polyline Z range overlaps stock Z range
//
// Recognize:
//   Metadata replay only (signature.params).

#include "Skill.hpp"

#include <array>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tube_hydroform {

constexpr const char* kSkillId = "tube_hydroform";

struct Input
{
    // Each entry: { axial_z_mm, outer_radius_mm }.  Must be Z-monotone
    // (strictly increasing) with at least two vertices.
    std::vector<std::array<double, 2>>  target_profile_polyline;
    double                              pressure_mpa = 200.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tube_hydroform
}  // namespace koocadcam::skill
