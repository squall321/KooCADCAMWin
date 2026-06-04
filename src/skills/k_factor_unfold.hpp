#pragma once
// @lat: [[engine/skills#k_factor_unfold]]
//
// k_factor_unfold — Unfold a bent sheet-metal part back to a flat blank using
// the K-factor (neutral-axis position).
//
//                  ╲                                  flat blank
//                   ╲   ←  bent (input)               ────────────
//                    ╲              =>                ────────────
//                   ─────── flat zone                 (developed length
//                                                      preserves the neutral
//                                                      axis material area)
//
// K-factor models the location of the neutral axis (where neither compression
// nor tension occurs) inside the bend:
//   K = t_neutral / t_sheet     ∈ [0.3, 0.5] for common metals
//   (0.33 mild steel, 0.42 aluminum, 0.50 soft copper)
//
// Developed bend allowance (BA) for an arc of inner radius R, angle α (rad),
// thickness t:
//   BA = α · (R + K · t)
//
// Synthesis strategy:
//   1. Split input shape at the bend axis into two halves (fixed + rotated).
//   2. Apply the INVERSE rotation to the rotated half (about bend_axis).
//   3. Translate it along the un-fold direction so the developed-length
//      arc material is reinserted as an extra flat slab whose length =
//      BA - chord_length (geometric correction).
//   4. Fuse → single planar flat blank.
//
// DFM:
//   DFM-K-SHEET    : 0.5 ≤ sheet_thickness ≤ 6.0 mm
//   DFM-K-FACTOR   : 0.3 ≤ k_factor ≤ 0.5
//   DFM-K-ANGLE    : 0 < |bend_angle_deg| ≤ 180
//
// Recognize:
//   A flat sheet with NO horizontal-axis cylindrical bend faces and an
//   accumulated developed-length signature beyond the original chord.

#include "Datum.hpp"
#include "Skill.hpp"

#include <array>
#include <memory>
#include <vector>

#include <gp_Ax1.hxx>

namespace koocadcam::skill {

class Workpiece;

namespace k_factor_unfold {

constexpr const char* kSkillId = "k_factor_unfold";

struct Input
{
    // bend axis: anchor + direction (unit vector); typically lies on inner
    // bend surface (z = sheet bottom).
    std::array<double, 3> bend_axis_origin   { 0.0, 0.0, 0.0 };
    std::array<double, 3> bend_axis_dir      { 0.0, 1.0, 0.0 };
    double                bend_angle_deg     = 90.0;
    double                sheet_thickness_mm = 1.5;
    double                k_factor           = 0.42;   // aluminum default
    double                inner_radius_mm    = 1.0;    // inner bend radius
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace k_factor_unfold
}  // namespace koocadcam::skill
