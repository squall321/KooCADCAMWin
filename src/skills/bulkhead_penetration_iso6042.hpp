#pragma once
// @lat: [[engine/skills#bulkhead_penetration_iso6042]]
//
// bulkhead_penetration_iso6042 (slice 16) — REAL compound feature for a
// cable/pipe penetration through a marine bulkhead with a fire-seal ring
// on BOTH faces (entry and exit), per ISO 6042-style marine fire-rated
// transit modules.
//
// Sub-features (single output shape) — built SEQUENTIALLY:
//   1. central through-bore (penetration_dia_mm × bulkhead_thickness_mm)
//   2. entry-face annular fire-seal pocket (deeper, fire_seal_ring_dia_mm)
//   3. exit-face  annular fire-seal pocket (mirror of step 2)
//
// CRITICAL — slice-13/14/15 root cause: do NOT compound the two fire-seal
// annular tools with the bore — OCCT 8.0 silently no-ops if their convex
// hulls overlap.  Use SEQUENTIAL pr::cut calls.
//
// DFM:
//   - dimensions positive.
//   - bulkhead_thickness ≥ 3 mm (marine hull plate minimum).
//   - fire_seal_ring_dia > penetration_dia + 4 mm (annular seal width ≥ 2 mm).
//   - fire_seal_depth ≤ bulkhead_thickness / 2 - 0.5 mm (so the two
//     opposing pockets cannot meet in the middle).
//
// Recognize:
//   - One axisymmetric central cylinder (the penetration bore).
//   - Two coaxial annular planar/cylindrical face groups, one at top one at
//     bottom — fire-seal annular pockets.
//   Confidence 0.8 (geometric) / 1.0 (replay).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bulkhead_penetration_iso6042 {

constexpr const char* kSkillId = "bulkhead_penetration_iso6042";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm           = 0.0;
    double      center_y_mm           = 0.0;
    gp_Dir      axis_dir              { 0.0, 0.0, 1.0 };
    double      penetration_dia_mm    = 32.0;
    double      bulkhead_thickness_mm = 12.0;
    double      fire_seal_ring_dia_mm = 60.0;
    double      fire_seal_depth_mm    = 4.0;
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bulkhead_penetration_iso6042
}  // namespace koocadcam::skill
