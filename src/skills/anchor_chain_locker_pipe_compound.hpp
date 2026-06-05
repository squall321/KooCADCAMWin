#pragma once
// @lat: [[engine/skills#anchor_chain_locker_pipe_compound]]
//
// anchor_chain_locker_pipe_compound (slice 16) — REAL compound feature for
// a marine anchor chain locker pipe with a grating support ring recess at
// a configurable height (where the chain gypsy grate rests).
//
// Sub-features (single output shape) — built SEQUENTIALLY:
//   1. central pipe through-bore (pipe_outer_dia × pipe_length)    [pr::cut]
//   2. annular grating support ring recess at grating_ring_position_z
//                                                                    [pr::cut]
//
// CRITICAL — slice-13/14/15 root cause: the grating ring recess overlaps
// the through-bore in radius range — do NOT cut as a compound; use
// SEQUENTIAL pr::cut.
//
// DFM:
//   - dimensions positive.
//   - pipe_wall_thickness ≥ 3 mm (marine pipe minimum).
//   - grating_ring_dia > pipe_outer_dia (support ring sits in pipe wall).
//   - grating_ring_position_z within [0, pipe_length].
//
// Recognize:
//   - One central +Z axisymmetric cylindrical bore.
//   - One coaxial larger cylinder at intermediate height — grating recess.
//   Confidence 0.8 (geometric) / 1.0 (replay).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace anchor_chain_locker_pipe_compound {

constexpr const char* kSkillId = "anchor_chain_locker_pipe_compound";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm                = 0.0;
    double      center_y_mm                = 0.0;
    gp_Dir      axis_dir                   { 0.0, 0.0, 1.0 };
    double      pipe_outer_dia_mm          = 250.0;
    double      pipe_wall_thickness_mm     = 12.0;
    double      pipe_length_mm             = 80.0;             // = bulkhead/stock thickness
    double      grating_ring_position_z_mm = 40.0;             // from bottom of pipe
    double      grating_ring_dia_mm        = 280.0;            // OD of recess (wider than pipe OD)
    double      grating_ring_depth_mm      = 6.0;              // axial recess depth
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace anchor_chain_locker_pipe_compound
}  // namespace koocadcam::skill
