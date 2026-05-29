#pragma once
// @lat: [[engine/skills#magnetic_latch_pocket]]
//
// magnetic_latch_pocket — COMPOUND CABINET-HARDWARE FEATURE.
//
// A standard "push-to-open" magnetic latch / catch mounts into a
// cylindrical pocket sized for a Ø8 mm neodymium magnet (slip fit, depth
// 5 mm), with a 0.5 mm retention chamfer at the rim (prevents magnet
// fall-out) and TWO M3 mounting-screw pilot holes spaced 30 mm apart.
//
// Sub-feature chain (4 primitive cuts):
//   1. Ø8 mm × 5 mm magnet pocket          (slip-fit blind cylinder)
//   2. 45° retention chamfer at rim        (cone frustum, top 0.5 mm)
//   3. left  M3 mounting pilot             (Ø2.5 mm × 6 mm)
//   4. right M3 mounting pilot             (Ø2.5 mm × 6 mm)
//
//      ◎───── 30 mm ─────◎       ◎ = M3 mounting pilots
//          ╱─────╲
//         │ Ø 8   │                  magnet pocket
//          ╲─────╱
//
// Standards:
//   - Magnet diameter: 8 mm Ø — Eclipse N48 / K&J catalog standard
//   - Slip fit: tolerance H10 ≈ +0.058 mm  → bore at 8.05 mm
//   - Mounting pitch: 30 mm (Sugatsune / Hettich catalog)
//
// DFM gates:
//   DFM-INPUT       : position finite
//   DFM-MAG-FIT     : magnet slip-fit clearance in range
//   DFM-MAG-DEPTH   : workpiece thickness ≥ magnet_depth + 2 mm

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace magnetic_latch_pocket {

constexpr const char* kSkillId = "magnetic_latch_pocket";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm  = 0.0;
    double      position_y_mm  = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace magnetic_latch_pocket
}  // namespace koocadcam::skill
