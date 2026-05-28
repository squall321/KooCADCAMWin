#pragma once
// @lat: [[engine/skills#metal_spin]]
//
// metal_spin — metal spinning: a circular disc blank is clamped against a
// rotating mandrel and progressively formed against the mandrel by a hand-
// held or CNC roller.  The result is an axisymmetric hollow shell whose
// inner profile mirrors the mandrel.
//
//                ┌──┐         roller
//                │░░│ ─────────────►
//          ╱─────┘░░└─────╲          (disc thins + flows over mandrel)
//         ╱     mandrel    ╲
//        ╱                  ╲
//        ──────axis (Z)──────
//
// Synthesis (geometric):
//   Similar to form_turn — we treat the input polyline of (z, r) waypoints
//   as the OUTER profile of the spun shell and revolve it 360° about Z to
//   obtain the finished workpiece.  Unlike form_turn (which CUTS material
//   from a billet) metal spinning REPLACES the entire workpiece with the
//   newly-formed shell — the disc volume is conserved while the radial
//   distribution is rearranged.
//
// Signature pattern (metadata-rich, geometry follows revolve):
//   - kind = metal_spin, axis = Z
//   - profile = the (z, r) polyline that was spun
//   - wall_thinning_pct — average thinning measured against the blank
//     thickness (≈ 1 − (r_blank / r_max)) — a forming-quality metric
//
// DFM checks:
//   DFM-INPUT     : ≥ 2 polyline points
//   DFM-INPUT     : z monotonic (strictly increasing)
//   DFM-INPUT     : every radius > 0.5 mm (forming-tool contact minimum)
//   DFM-SPIN-THIN : implied wall_thinning_pct > 70% (excessive thinning
//                   → high risk of tearing; spin in multiple stages)
//   DFM-SPIN-ANG  : implied flange angle > 80° (near-flat spins require
//                   shear-spinning rather than conventional spinning)
//
// Recognize:
//   (a) Metadata replay — exact recovery of the polyline + thinning.
//   (b) Geometric fallback — detect a Z-axis cylindrical or conical face
//       and emit a coarse 2-point profile with low confidence.

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace metal_spin {

constexpr const char* kSkillId = "metal_spin";

struct ProfilePoint
{
    double z_mm = 0.0;
    double r_mm = 0.0;
};

struct Input
{
    // Polyline of (z, r) waypoints, sorted by z ascending, ≥ 2 points.
    // Defines the OUTER profile of the spun shell.
    std::vector<ProfilePoint>  profile;

    // Blank disc thickness (used to estimate wall_thinning_pct).
    double                     blank_thickness_mm = 1.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace metal_spin
}  // namespace koocadcam::skill
