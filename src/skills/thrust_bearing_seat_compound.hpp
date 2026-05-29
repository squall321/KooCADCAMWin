#pragma once
// @lat: [[engine/skills#thrust_bearing_seat_compound]]
//
// thrust_bearing_seat_compound — COMPOUND axial-thrust bearing seat:
//
//   sub 1: shaft-end face turning (face cut to depth = face_depth_mm)
//   sub 2: raceway ring groove (annular ring for thrust washer)
//   sub 3: lock-nut pilot diameter (smaller cylindrical step for retaining
//          nut threads — geometry only; thread is metadata)
//   sub 4: thrust shoulder (the planar surface the thrust race bears against)
//
// All sub-features concentric on `axis_dir`.  The face cut & shoulder are
// achieved as a single stepped cylinder cut; the raceway groove is an
// annular ring fused into the tool stack.
//
//                       face cut (entry plane)
//                              │
//             ┌────────────────▼─────────────────┐
//             │             ───────── outer_dia ─────
//             │  raceway ╗      lock-nut pilot
//             │  groove  ╗
//             │             ───────── inner_dia ─────
//             └────────────────────────────────────┘
//                              ▲
//                       thrust shoulder
//
// Snap-ring style ring grooves are intentionally NOT used here — the
// raceway groove follows the thrust-bearing raceway standard (Timken
// thrust series: groove width = roller_dia × 1.1, groove depth = roller_dia
// × 0.4).  Slice 9 fixes nominal values:
//   raceway_groove_width  = (outer_dia − inner_dia) / 6
//   raceway_groove_depth  = (outer_dia − inner_dia) / 8
//
// DFM checks:
//   DFM-INPUT       : non-positive dims, axis unset
//   DFM-SEAT-GEOM   : inner_dia ≥ outer_dia
//   DFM-002         : (outer_dia − inner_dia) < 1.6 mm  (insufficient race
//                     width for a thrust bearing)
//   DFM-LOCKNUT     : inner_dia < 3.0 mm (M3 minimum lock-nut shaft thread)
//
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace thrust_bearing_seat_compound {

constexpr const char* kSkillId = "thrust_bearing_seat_compound";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      outer_dia_mm  = 0.0;
    double      inner_dia_mm  = 0.0;
    double      face_depth_mm = 1.0;     // shaft-end face cut (must be > 0)
    double      seat_depth_mm = 4.0;     // axial extent of the seat region
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace thrust_bearing_seat_compound
}  // namespace koocadcam::skill
