#pragma once
// @lat: [[engine/skills#manifold_block_compound]]
//
// manifold_block_compound — hydraulic manifold with main bore + N branch
// BSP G-tapped ports (HVAC / fluid).
//
// Slice 15 — HVAC/fluid.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. main bore (cylinder cut along main_axis_dir, main_length deep)
//   2..(1+N). N perpendicular branch bores, spaced by branch_pitch along the
//             main axis.  Each branch:
//              (a) straight drill at BSP tap_drill_dia
//              (b) parallel spotface dish at the entry face
//             — pre-fused (concentric drill + spotface) then cut, mirroring
//             the slice-13 BSP G port pattern.
//
// subfeature_count = 1 + branch_count.
//
// DFM:
//   DFM-BSP-CODE   : branch_thread_size_key not in central BSP G table.
//   DFM-INPUT      : geometric inputs must be > 0.
//   DFM-BRANCH-FIT : main_length too short for N branches × pitch.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace manifold_block_compound {

constexpr const char* kSkillId = "manifold_block_compound";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      main_bore_origin   { 0.0, 0.0, 0.0 };
    gp_Dir      main_axis_dir      { 1.0, 0.0, 0.0 };   // main bore along +X
    double      main_dia_mm        = 20.0;
    double      main_length_mm     = 120.0;
    int         branch_count       = 4;
    double      branch_dia_mm      = 8.0;
    double      branch_pitch_mm    = 25.0;
    std::string branch_thread_size_key;                 // BSP G e.g. "G1/4"
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace manifold_block_compound
}  // namespace koocadcam::skill
