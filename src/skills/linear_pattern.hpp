#pragma once
// @lat: [[engine/skills#linear_pattern]]
//
// linear_pattern — SolidWorks/Fusion linear pattern of a hole.
//
// Replicate a cylindrical cut (count_x × count_y) along an XY grid.
//
//   start_x_mm + i*pitch_x_mm   (i ∈ [0, count_x))
//   start_y_mm + j*pitch_y_mm   (j ∈ [0, count_y))
//
// apply():  for each (i,j) build pr::cylinder along -Z axis, bundle them all
//           into a single pr::cutMany against wp.shape().
//
// DFM gates:
//   DFM-INPUT  : count_x ≥ 1, count_y ≥ 1, hole_dia_mm > 0, hole_depth_mm > 0
//   DFM-PATT-1 : pitch_x_mm > hole_dia_mm  (no overlap)
//   DFM-PATT-1 : pitch_y_mm > hole_dia_mm  (when count_y > 1)
//   DFM-002    : hole_dia_mm ≥ 0.8 mm (ISO 235 smallest standard drill)
//
// Recognize:
//   Walk cylindrical faces along -Z, group by axis direction + radius into a
//   collinear/colinear-grid set.  If ≥ 2 identical cylinders share a constant
//   pitch, surface a candidate.

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace linear_pattern {

constexpr const char* kSkillId = "linear_pattern";

struct Input
{
    FaceDatum   face;                              // host face (top, +Z normal)
    double      hole_dia_mm    = 0.0;
    double      hole_depth_mm  = 0.0;
    int         count_x        = 1;
    double      pitch_x_mm     = 0.0;
    int         count_y        = 1;
    double      pitch_y_mm     = 0.0;
    double      start_x_mm     = 0.0;
    double      start_y_mm     = 0.0;

    // Direct world placement (the foreign-recovery path, like box_boss).  When
    // `use_world` is set the grid is laid out about `world_origin` in the plane
    // whose OUTWARD normal is `world_axis`, instead of resolving `face`.  The
    // (start/pitch) offsets are the same face-local (u,v) as the datum path, so a
    // side grille (axis along +/-X or +/-Y) replays in place.
    bool        use_world    = false;
    double      world_ox_mm = 0.0, world_oy_mm = 0.0, world_oz_mm = 0.0;  // grid origin
    double      world_nx = 0.0, world_ny = 0.0, world_nz = 1.0;          // outward normal
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace linear_pattern
}  // namespace koocadcam::skill
