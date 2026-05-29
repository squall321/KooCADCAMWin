#pragma once
// @lat: [[engine/skills#radial_bearing_seat_with_snapring]]
//
// radial_bearing_seat_with_snapring — COMPOUND bearing seat:
//
//   sub 1: outer bore at outer_dia (bearing seat OD)
//   sub 2: inner shoulder step at inner_dia (axial retention)
//   sub 3: external snap-ring groove (DIN 471 dimensions)
//   sub 4: 45° lead-in chamfer at entry
//
// Topology after apply (visualised with axis horizontal, entry on the left):
//
//       45° chamfer
//          │            ┌──── snap-ring groove (DIN 471)
//          ▼            ▼
//        ╲┌─────────────┬────────┐
//   ─────│ outer bore  │ shoulder│           bearing ID region
//        └─────────────┴────────┘
//                       │
//                       └─ inner_dia step
//
// All sub-features share the same axis (`axis_dir`) and origin (entry on the
// `entry_face` at xy = position_x_mm, position_y_mm).  They are concentric
// overlapping cylinders + a ring, fused into a single tool, then ONE
// boolean cut against the workpiece (`fuse_first_then_cut`).
//
// Snap-ring standard (DIN 471): groove diameter d2 ≈ outer_dia + 0.7 × t,
// groove width w ≈ 1.5 × t, where t is the snap-ring thickness selected
// from the DIN 471 nominal-bore table:
//   outer_dia 10-18 mm → t = 1.0 mm
//   outer_dia 18-30 mm → t = 1.2 mm
//   outer_dia 30-50 mm → t = 1.5 mm
//   outer_dia 50-80 mm → t = 2.0 mm
//
// DFM checks:
//   DFM-INPUT          : outer_dia ≤ 0, inner_dia ≤ 0, depth ≤ 0
//   DFM-SEAT-GEOM      : inner_dia ≥ outer_dia (no shoulder step)
//   DFM-002            : inner_dia < 0.8 mm (sub-min drill / mill)
//   DFM-DIN471-RANGE   : outer_dia outside DIN 471 supported range (10…80 mm)
//   DFM-DIN471-DEPTH   : depth_mm < groove_axial_position + 2 × groove width
//
// Recognize:
//   Metadata replay only (compound features have overlapping topology with
//   plain counterbore + ring groove; geometric fingerprinting is unreliable).
//   Confidence 1.0 from `wp.features()`.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace radial_bearing_seat_with_snapring {

constexpr const char* kSkillId = "radial_bearing_seat_with_snapring";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm   = 0.0;
    double      position_y_mm   = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };
    double      outer_dia_mm    = 0.0;   // bearing OD == bore size
    double      inner_dia_mm    = 0.0;   // shoulder ID (must be < outer_dia)
    double      depth_mm        = 0.0;   // total axial depth of the seat
    std::string snap_ring_std   = "DIN471";  // currently only DIN 471 supported
};

// DIN 471 snap-ring thickness for a given bore diameter.  Returns 0.0 if
// outside the supported 10…80 mm bore range.
double snapRingThicknessFor(double outer_dia_mm);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace radial_bearing_seat_with_snapring
}  // namespace koocadcam::skill
