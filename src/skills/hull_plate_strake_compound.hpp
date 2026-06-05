#pragma once
// @lat: [[engine/skills#hull_plate_strake_compound]]
//
// hull_plate_strake_compound (slice 16) — REAL multi-cut compound feature
// for a marine hull strake (rolled plate edge with overlap rivet line).
//
// Sub-features (single output shape) — built SEQUENTIALLY:
//   1. edge-relief chamfer/recess along the strake edge          [pr::cut]
//   2. N rivet holes drilled along the strake axis at rivet_pitch
//                                                                 [pr::cut × N
//                                                                  SEQUENTIAL —
//                                                                  do NOT cutMany]
//
// CRITICAL — slice-13/14/15 root cause: rivet holes packed tightly along
// the strake edge will have overlapping convex hulls if drilled as a
// compound; OCCT 8.0 silently no-ops.  Use SEQUENTIAL pr::cut.
//
// DFM:
//   - dimensions positive.
//   - rivet_pitch ≥ 3 × rivet_dia (Lloyd's Register hull-plate rivet pitch
//     minimum).
//   - rivet_count ≥ 2.
//   - edge_radius ≥ 0 (0 = no edge relief).
//
// Recognize:
//   - A linear row of ≥ 2 small coaxial cylinder faces (the rivets) on a
//     hull plate.
//   - Optional: an edge relief box/recess along the strake axis.
//   Confidence 0.75 (geometric) / 1.0 (replay).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hull_plate_strake_compound {

constexpr const char* kSkillId = "hull_plate_strake_compound";

struct Input
{
    FaceDatum   entry_face;
    gp_Pnt      strake_axis_origin    { 0.0, 0.0, 0.0 };   // start of rivet line
    gp_Dir      strake_axis_dir       { 1.0, 0.0, 0.0 };   // direction of rivet line (in plate plane)
    gp_Dir      drill_axis_dir        { 0.0, 0.0, 1.0 };   // direction rivets drill (plate normal)
    double      strake_length_mm      = 200.0;
    double      rivet_pitch_mm        = 40.0;
    double      rivet_dia_mm          = 8.0;
    int         rivet_count           = 5;
    double      edge_radius_mm        = 1.5;             // optional edge relief
    double      plate_thickness_mm    = 10.0;
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hull_plate_strake_compound
}  // namespace koocadcam::skill
