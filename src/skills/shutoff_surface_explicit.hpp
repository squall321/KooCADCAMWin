#pragma once
// @lat: [[engine/skills#shutoff_surface_explicit]]
//
// shutoff_surface_explicit — Explicit shut-off surface where the moving and
// stationary mold halves meet to seal an undercut.  Given the polyline edge
// of the undercut, we extrude a quadrilateral strip face along the mold pull
// direction by `shutoff_face_size_mm` and fuse it onto the workpiece as a
// thin solid (the shut-off geometry that closes the undercut gap).
//
//      undercut_edge_polyline (vector<gp_Pnt>)
//      P0 ───────── P1 ───────── P2 ───────── P3
//                          │  pull_dir_xyz   │
//                          ▼                 ▼
//      P0'───────── P1'───────── P2'───────── P3'    (offset by shutoff_face_size)
//
// Implementation:
//   1. Build a thin extruded ribbon by sweeping each polyline segment along
//      pull_dir for shutoff_face_size_mm.  Each segment becomes a box
//      (length × tiny_thickness × shutoff_face_size_mm) — the resulting
//      solid is the shut-off "wall" that closes the undercut.
//   2. Fuse all ribbon boxes onto the workpiece.
//
// Signature pattern:
//   pattern.kind                    = "shutoff_surface_explicit"
//   pattern.is_compound             = true
//   pattern.mold_feature_type       = "shutoff_surface"
//   pattern.derived_volume_removed  = -fused_volume   (negative because added)
//
// DFM:
//   DFM-INPUT                       : polyline ≥ 2 points;
//                                     shutoff_face_size_mm > 0.
//   DFM-SHUTOFF-CLEARANCE           : shutoff_face_size_mm ≥ 0.5 mm.

#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace shutoff_surface_explicit {

constexpr const char* kSkillId = "shutoff_surface_explicit";

struct Input
{
    std::vector<gp_Pnt>  undercut_edge_polyline;
    double               pull_dir_x          = 0.0;
    double               pull_dir_y          = 0.0;
    double               pull_dir_z          = 1.0;
    double               shutoff_face_size_mm = 2.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace shutoff_surface_explicit
}  // namespace koocadcam::skill
