#pragma once
// @lat: [[engine/skills#linear_rail_seat_compound]]
//
// linear_rail_seat_compound — COMPOUND MACRO for a linear-motion rail seat.
// Combines:
//   1. A longitudinal slot (rail recess) along the rail centerline.
//   2. Two parallel rows of M5 tapped mounting holes (at fixed pitch).
//   3. A peripheral edge chamfer at the seat opening (entry-rail-edge).
//
//              ↓ rail axis (centerline)
//        ┌───────────────────────────────────┐
//        │   ○    ○    ○    ○    ○    ○    │  ← row A holes (pitch)
//        │ ┌─────────────────────────────┐ │
//        │ │       rail slot              │ │
//        │ └─────────────────────────────┘ │
//        │   ○    ○    ○    ○    ○    ○    │  ← row B holes (pitch)
//        └───────────────────────────────────┘
//
// Sub-feature count = 1 slot + 2N holes + 1 chamfer (>= 5 for typical 2-hole/row layout).
//
// Standards reference: DIN 645 / ISO 12090 for LM-rail mounting hole pattern,
// pitch typically 20–60 mm for size 15–25 mm rails.
//
// DFM checks:
//   DFM-INPUT       : length, pitch, rail_centerline > 0
//   DFM-002         : M5 pilot dia (4.2 mm) auto-satisfied (>= 0.8 mm)
//   DFM-RAIL-PITCH  : pitch_mm in [10, 80] mm (warning outside)
//   DFM-RAIL-LEN    : at least one full pitch fits in length
//   DFM-RAIL-WALL   : rail-row offset from slot edge >= 2 mm
//
// Recognize: metadata-only (compound features are not fingerprinted by B-rep).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace linear_rail_seat_compound {

constexpr const char* kSkillId = "linear_rail_seat_compound";

struct Input
{
    FaceDatum   entry_face;
    double      length_mm           = 0.0;   // slot length along rail axis
    double      rail_centerline_mm  = 0.0;   // Y position of rail centerline (XY plane)
    double      pitch_mm            = 0.0;   // mounting hole pitch along rail axis
    // Derived defaults (not on input, but documented):
    //   slot_width_mm  = 12.0 mm (typical LM-rail recess width)
    //   slot_depth_mm  =  4.0 mm
    //   row_offset_mm  =  9.0 mm (each side from centerline)
    //   tap_depth_mm   =  8.0 mm (M5 standard)
    //   chamfer_size_mm = 0.5 mm
    double      start_x_mm          = 0.0;   // X position of first hole / slot start
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace linear_rail_seat_compound
}  // namespace koocadcam::skill
