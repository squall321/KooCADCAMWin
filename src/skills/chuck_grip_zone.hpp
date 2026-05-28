#pragma once
// @lat: [[engine/skills#chuck_grip_zone]]
//
// chuck_grip_zone — metadata-only.  Tags a CYLINDRICAL zone of the
// workpiece (along a given axis, of a given length) as the "grip zone"
// where a lathe chuck (3-jaw, 4-jaw, collet) is holding the part.
// Subsequent operations consult this metadata so toolpaths can be
// constrained NOT to enter the grip zone — both for collision safety
// (jaws would foul the tool) and for surface-integrity safety (jaws crush
// finish surfaces).
//
//      ────────────────────────────────────────────────────────────
//      │ ░░░░░░░░░░ grip zone ░░░░░░░░░░ │   exposed cut region   │
//      │   (chuck contact, no machining)  │   (free for ops)       │
//      ────────────────────────────────────────────────────────────
//        ←─── grip_length ────→
//                  ║
//             cylinder_axis (gp_Ax1 — origin + direction)
//
// Signature pattern:
//   - pattern.kind             = "chuck_grip_zone"
//   - pattern.cylinder_axis    = { origin:[x,y,z], direction:[dx,dy,dz] }
//   - pattern.grip_length      = <mm>
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT       grip_length <= 0                       (error)
//   CG-AXIS-NORM    |direction| not finite or zero          (error)
//   CG-LENGTH-SHORT grip_length < 3 mm                      (warning — typical
//                   minimum chuck-jaw engagement is ≥ 3 mm for stability)
//
// Recognize: metadata replay only.

#include "Skill.hpp"

#include <gp_Ax1.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace chuck_grip_zone {

constexpr const char* kSkillId = "chuck_grip_zone";

struct Input
{
    gp_Ax1  cylinder_axis;    // grip cylinder's central axis
    double  grip_length  = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace chuck_grip_zone
}  // namespace koocadcam::skill
