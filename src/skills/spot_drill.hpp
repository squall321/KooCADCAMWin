#pragma once
// @lat: [[engine/skills#spot_drill]]
//
// spot_drill — a SHALLOW conical mark used as a centering reference before a
// real drill_hole.  The spot drill creates a small cone seat that "locks" the
// follow-up twist drill's tip so the bore starts true at the intended XY.
//
//                          ┌─ entry_face (planar)
//                          ▼
//                    ──────┬───────────┬──────
//                          │\         /│
//                          │ \   α   / │           ↑ depth_mm
//                          │  \     /  │           │   (derived)
//                          │   \   /   │           │
//                          │    \ /    │           ↓
//                          └─────V─────┘
//                             apex
//                          ←── dia ──→
//
// Derived geometry:
//   depth_mm = (diameter / 2) / tan(cone_angle / 2)
//
// Signature pattern (what the skill leaves on the workpiece):
//   - 1 conical face (the cone seat, apex on workpiece interior)
//   - 1 circular edge at the entry face (cone top, large circle)
//   - The cone tip is a single vertex (no closing planar face)
//
// DFM checks:
//   DFM-002 : diameter_mm ≥ 0.8 mm  (standard spot-drill min size)
//   cone_angle_deg ∈ [60, 140]      (typical 90° / 120° tools; outside this
//                                    is non-standard and gets blocked).
//
// Recognize:
//   Iterate conical faces.  A spot mark is an UNPAIRED cone (no coaxial
//   cylinder for a countersink, no cylinder-then-cone for any drill seat),
//   relatively shallow (depth < diameter), and its cone's smaller circle
//   degenerates near a single point (apex inside material).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spot_drill {

constexpr const char* kSkillId = "spot_drill";

struct Input
{
    FaceDatum   entry_face;                       // which face is being spotted
    double      position_x_mm  = 0.0;             // world XY of the cone center
    double      position_y_mm  = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };// drilling direction (into the material)
    double      diameter_mm    = 0.0;             // cone-top diameter at entry face
    double      cone_angle_deg = 90.0;            // included apex angle (typ. 90° / 120°)
};

// Synthesis: apply this skill to a workpiece.
// Returns a new workpiece (cloned with the spot cone cut) + the FeatureSignature.
SkillOutput apply(const Workpiece& wp, const Input& in);

// DFM validation.
DFMReport validate(const Workpiece& wp, const Input& in);

// Recognize candidates of this skill in the given workpiece.
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Derived geometry: depth = (dia/2) / tan(angle/2).  Returns 0 on degenerate input.
double computeDepth(const Input& in);

}  // namespace spot_drill
}  // namespace koocadcam::skill
