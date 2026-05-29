#pragma once
// @lat: [[engine/skills#spring_loaded_door_latch]]
//
// spring_loaded_door_latch — pocket cluster for a spring-loaded door latch
// (the kind on cabinet doors / appliance doors).  Five sub-features:
//
//   1. Latch body pocket — square pocket sized to receive the latch body.
//   2. 2 retention screw holes — countersunk M3 screws clamping the body.
//   3. Plunger bore — horizontal-axis cylinder through the latch body
//      pocket back wall for the spring-loaded plunger to extend through.
//   4. Receiver slot — rectangular cut on the door-strike side that lets
//      the plunger nose protrude past the workpiece face when engaged.
//   5. (implicit fuse cleanup — the four cuts merge into one signature.)
//
// REAL geometry chain: 4+ Boolean ops (body pocket + 2 screws + plunger
// bore + receiver slot, fused into one cutter, single cut).
//
// Standard:
//   - ANSI/BHMA A156.16 (auxiliary cabinet latches): typical body 18 × 18 mm,
//     plunger ø6 mm, mount screw M3, screw spacing 25 mm.
//   - DIN 7991 countersunk head dimensions.
//
// DFM gates:
//   - latch_pocket_size_mm  ≥ 6 mm (functional minimum).
//   - screw_hole_dia_mm     ≥ 0.8 mm (DFM-002).
//   - plunger_bore_dia_mm   ≥ 1.6 mm.
//   - receiver_slot_width_mm ≥ plunger_bore_dia_mm (must let plunger pass).
//   - screw_spacing_mm > 2 × screw_hole_dia_mm.
//   - latch_pocket_depth_mm + plunger_bore_depth_mm <= workpiece depth.
//
// Signature:
//   - is_compound = true
//   - subfeature_count = 5 (body pocket + 2 screws + plunger + receiver)
//
// Recognize:
//   - PRIMARY (geometric): find a square top-face pocket, 2 small Z-axis
//     cylinders inside it (the screw holes), and at least one off-axis
//     non-vertical cylinder (the plunger bore).
//   - FALLBACK (metadata replay): confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spring_loaded_door_latch {

constexpr const char* kSkillId = "spring_loaded_door_latch";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm        = 0.0;       // latch pocket center XY
    double      position_y_mm        = 0.0;
    gp_Dir      axis_dir             { 0.0, 0.0, -1.0 };
    double      latch_pocket_size_mm = 18.0;      // square pocket side length
    double      latch_pocket_depth_mm = 10.0;
    double      screw_hole_dia_mm    = 3.4;       // M3 clearance
    double      screw_hole_depth_mm  = 12.0;
    double      screw_spacing_mm     = 25.0;      // along +X centerline
    double      plunger_bore_dia_mm  = 6.0;
    double      plunger_bore_depth_mm = 12.0;     // protrudes through back wall
    double      receiver_slot_width_mm  = 7.0;
    double      receiver_slot_height_mm = 3.0;
    double      receiver_slot_depth_mm  = 4.0;    // depth into workpiece on -X
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace spring_loaded_door_latch
}  // namespace koocadcam::skill
