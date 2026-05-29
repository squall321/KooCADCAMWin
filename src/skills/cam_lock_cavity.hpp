#pragma once
// @lat: [[engine/skills#cam_lock_cavity]]
//
// cam_lock_cavity — COMPOUND CABINET-LOCK FEATURE.
//
// The classic "cam lock" used on cabinet doors, postal boxes, and locker
// doors mounts in a Ø19 mm body bore.  Below the keyway face there's a
// radial slot that the rotating cam-arm sweeps through, plus a shallow
// counterbore on the keyway side that recesses the decorative cap.
//
// Sub-feature chain (3 primitive cuts):
//   1. Ø19 mm lock-body through-bore         (cylinder, full thickness)
//   2. radial cam-arm slot                   (box, length 35 mm × height 6 mm,
//                                             at body bottom on inboard side)
//   3. keyway-side counterbore for cap       (Ø22 mm × 1.5 mm)
//
//                  ◯ Ø 22 cap counterbore (key side)
//                  │
//             ─────┴────  Ø 19  body bore (through)
//                  │
//                  │
//                  │
//             ─────┴────  cam slot (35 × 6) at far face
//
// Standards:
//   - Body OD: 19 mm (Master Lock / Sentry-X cam-lock catalog standard)
//   - Cap recess: 22 mm Ø × 1.5 mm depth (covers decorative bezel)
//   - Cam slot:  35 mm × 6 mm  (clears a 30 mm cam swing)
//
// DFM gates:
//   DFM-INPUT      : position finite
//   DFM-LOCK-BODY  : workpiece thickness ≥ 20 mm (covers 19 mm body + cap)
//   DFM-LOCK-CAM   : cam slot must fit within workpiece YZ bbox

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cam_lock_cavity {

constexpr const char* kSkillId = "cam_lock_cavity";

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

}  // namespace cam_lock_cavity
}  // namespace koocadcam::skill
