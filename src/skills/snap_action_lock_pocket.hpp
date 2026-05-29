#pragma once
// @lat: [[engine/skills#snap_action_lock_pocket]]
//
// snap_action_lock_pocket — pocket cluster for a cam-lock / snap-action
// cylinder lock body, with the supporting key counterbore, cam-arm slot,
// and retention spring pocket.
//
// REAL geometry chain (4 cuts):
//   1. Lock body cylindrical bore — main bore for the lock cylinder
//      (lock_body_dia × lock_body_depth).
//   2. Key counterbore — larger-diameter shallow seat at the entry face for
//      the key escutcheon (concentric with lock body bore).
//   3. Cam-arm slot — narrow rectangular slot along the cam axis to let the
//      cam arm rotate INSIDE the lock body cavity (tangent to bore wall).
//   4. Retention spring pocket — small cylindrical hole offset from the
//      lock body axis, drilled into the bore wall to seat the spring.
//
// Standard:
//   - DIN 18254 / DIN EN 12320 cam-lock geometry (commercial cabinet locks).
//   - ANSI/BHMA A156.5 (auxiliary locks): cam lock bore ø19 — ø25 mm typical.
//
// DFM gates:
//   - lock_body_dia_mm ≥ 6 mm (smallest commercial cam-lock).
//   - key_counterbore_dia_mm > lock_body_dia_mm AND
//     key_counterbore_depth_mm < lock_body_depth_mm  (else it's a through-bore).
//   - cam_slot_width_mm ≥ 0.8 mm (DFM-002).
//   - spring_pocket_dia_mm ≥ 0.8 mm.
//   - spring_pocket_dia_mm + 1.0 mm < (lock_body_dia_mm/2)  (else spring
//     pocket would break through bore wall).
//
// Signature:
//   - is_compound = true
//   - subfeature_count = 4
//
// Recognize:
//   - PRIMARY (geometric): find pair of coaxial cylinders (lock body +
//     counterbore) + at least one off-axis small cylinder (spring pocket)
//     + at least one rectangular slot (cam slot).
//   - FALLBACK (metadata replay): confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace snap_action_lock_pocket {

constexpr const char* kSkillId = "snap_action_lock_pocket";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm           = 0.0;
    double      position_y_mm           = 0.0;
    gp_Dir      axis_dir                { 0.0, 0.0, -1.0 };
    double      lock_body_dia_mm        = 19.0;     // ANSI BHMA A156.5 standard
    double      lock_body_depth_mm      = 22.0;
    double      key_counterbore_dia_mm  = 24.0;     // escutcheon seat
    double      key_counterbore_depth_mm = 2.0;
    double      cam_slot_length_mm      = 30.0;     // beyond bore — for arm swing
    double      cam_slot_width_mm       = 4.0;
    double      cam_slot_depth_offset_mm = 18.0;    // Z depth of slot center
    double      spring_pocket_dia_mm    = 2.5;
    double      spring_pocket_offset_mm = 6.0;      // distance from bore axis
    double      spring_pocket_depth_mm  = 6.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace snap_action_lock_pocket
}  // namespace koocadcam::skill
