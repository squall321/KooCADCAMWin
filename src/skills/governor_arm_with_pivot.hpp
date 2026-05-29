#pragma once
// @lat: [[engine/skills#governor_arm_with_pivot]]
//
// governor_arm_with_pivot — a centrifugal-governor pivot arm with:
//   1. An L-shaped (or straight-T) arm body fused onto the workpiece
//   2. A central pivot through-bore at the L vertex
//   3. Two spherical ball-seat pockets at the arm ends (where the
//      flyball linkage attaches via ball studs)
//
// Reference standards:
//   ASME B18.3 (socket-head ball bearing seat)
//   ANSI/ASME B5.16 (governor pivot dimensional standard, vintage but
//                    still used for ICE governor designs)
//
// Compound feature (≥ 4 cuts/fuses):
//   step 0: fuse L-arm — two perpendicular rectangular box bodies forming
//           the "L" planview, axially extending in +Z by arm_thickness.
//   step 1: CUT central pivot bore (axial, full through-thickness).
//   step 2: CUT ball-seat pocket at arm end 1 (cylindrical, partial depth).
//   step 3: CUT ball-seat pocket at arm end 2 (cylindrical, partial depth).
//
// DFM gate:
//   - arm_length_mm ≥ 10 mm (each leg of the L)
//   - arm_width_mm ≥ 4 mm (cross-section)
//   - arm_thickness_mm ≥ 3 mm
//   - pivot_bore_dia_mm ≥ 2 mm and ≤ arm_width_mm × 0.7
//   - ball_seat_dia_mm ≥ 2 mm and ≤ arm_width_mm × 0.8
//   - ball_seat_depth_mm ≤ arm_thickness_mm × 0.7 (not through)
//
// Recognition:
//   PRIMARY (geometric): one axial cylinder = pivot bore.  Two additional
//     axial cylinders (smaller, partial-depth) at offset positions =
//     ball-seat pockets.
//   FALLBACK (metadata).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace governor_arm_with_pivot {

constexpr const char* kSkillId = "governor_arm_with_pivot";

struct Input
{
    double arm_length_mm        = 40.0;   // length of each L-leg from the corner
    double arm_width_mm         = 8.0;    // cross-section width
    double arm_thickness_mm     = 6.0;    // axial thickness (Z)
    double pivot_bore_dia_mm    = 4.0;
    double ball_seat_dia_mm     = 5.0;
    double ball_seat_depth_mm   = 2.5;
    double corner_x_mm          = 0.0;    // pivot (L-corner) position
    double corner_y_mm          = 0.0;
    double corner_z_mm          = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace governor_arm_with_pivot
}  // namespace koocadcam::skill
