#pragma once
// @lat: [[engine/skills#counterbore]]
//
// counterbore — drill a pilot hole plus a larger-diameter cylindrical
// "seat" at the entry face, so the head of a socket-head cap screw can
// sit flush with (or below) the entry surface.
//
//                          ┌─ entry_face (planar)
//                          ▼
//                    ──────┬───────────┬──────
//                          │seat_dia   │
//                          │ ◄──────►  │
//                          │           │       ↑ seat_depth_mm
//                          │           │       │
//                          │   ┌───┐   │       ↓
//                          │   │   │   │       ↑
//                          │   │pilot_dia│             │
//                          │   │   │   │       │ pilot_depth_mm
//                          │   │   │   │       │
//                          │   └───┘   │       ↓
//                          └───────────┘
//
// Two coaxial cylindrical cuts:
//   • Pilot — small diameter, deep (full pilot_depth from entry face).
//   • Seat  — large diameter, shallow (seat_depth from entry face).
//
// Signature pattern (what the skill leaves on the workpiece):
//   - 2 coaxial cylindrical faces with different radii
//   - 1 annular planar face (the "step" at seat_depth that connects the
//     two cylinders)
//   - 1 bottom planar face (the pilot's blind bottom)
//   - circular edges at the entry face (seat top), the step (twice — outer
//     and inner of the annulus), and the pilot bottom
//
// DFM checks:
//   DFM-002          : pilot_dia_mm  ≥ 0.8 mm  (standard small endmill)
//   seat_dia_mm > pilot_dia_mm  (else it's not a counterbore)
//   seat_depth_mm < pilot_depth_mm (else it's not a counterbore)
//   pilot depth/dia ratio > 8 → warning (peck drilling required)
//
// Recognize:
//   Iterates pairs of coaxial cylindrical faces.  The pair where the
//   larger-radius face is adjacent to (i.e. shares a circular edge with)
//   the workpiece entry face is the counterbore seat; the smaller-radius
//   face is the pilot bore.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace counterbore {

constexpr const char* kSkillId = "counterbore";

struct Input
{
    FaceDatum   entry_face;                       // which face the screw head sits in
    double      position_x_mm = 0.0;              // common axis center, world XY
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };// drilling direction (into the material)
    double      pilot_dia_mm   = 0.0;             // small, deeper bore
    double      pilot_depth_mm = 0.0;             // measured from entry face
    double      seat_dia_mm    = 0.0;             // larger seat for the head
    double      seat_depth_mm  = 0.0;             // measured from entry face
};

// Synthesis: apply this skill to a workpiece.
// Returns a new workpiece (cloned with the counterbore cut) + the FeatureSignature.
SkillOutput apply(const Workpiece& wp, const Input& in);

// DFM validation.
DFMReport validate(const Workpiece& wp, const Input& in);

// Recognize candidates of this skill in the given workpiece.
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace counterbore
}  // namespace koocadcam::skill
