#pragma once
// @lat: [[engine/skills#countersink]]
//
// countersink — drill a pilot hole plus a CONICAL seat at the entry face
// so a flat-head screw can sit flush.  Unlike counterbore (which uses a
// cylindrical seat), countersink uses a cone.  Standard included angles
// are 82° (UNC) and 90° (metric).
//
//                          ┌─ entry_face (planar)
//                          ▼
//                    ──────┬───────────┬──────
//                          │\         /│
//                          │ \       / │           ↑ cone_depth (derived)
//                          │  \  α  /  │           │
//                          │   \   /   │           ↓
//                          │   │   │   │           ↑
//                          │   │   │   │           │
//                          │   │pilot_dia│             │ pilot_depth_mm
//                          │   │   │   │           │
//                          │   └───┘   │           ↓
//                          └───────────┘
//
// One cylindrical cut (pilot) plus one conical cut (countersink).
//
// Derived: cone_depth = (cone_top_dia − pilot_dia) / 2 / tan(cone_angle/2)
//
// Signature pattern (what the skill leaves on the workpiece):
//   - 1 cylindrical face (the pilot bore wall)
//   - 1 conical face on top, coaxial with the cylinder
//   - 1 circular edge at the entry face (the LARGER circle = cone top)
//   - 1 circular edge at the cone-cylinder junction
//   - 1 planar face at the pilot bottom (blind only)
//
// DFM checks:
//   DFM-002          : pilot_dia_mm ≥ 0.8 mm
//   cone_top_dia > pilot_dia          (else it's not a countersink)
//   45° ≤ cone_angle ≤ 120°           (sane included-angle range)
//   pilot depth/dia ratio > 8 → warning
//
// Recognize:
//   Iterates conical + cylindrical face pairs sharing the same axis.  The
//   cone's larger edge sits on the entry face, the smaller edge meets the
//   cylinder's top.  Recover pilot_dia (= 2×cyl.Radius), cone_top_dia
//   (= 2×coneMaxRadius at entry), cone_angle (= 2×cone.SemiAngle),
//   pilot_depth (= cyl.Axis projection length).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace countersink {

constexpr const char* kSkillId = "countersink";

struct Input
{
    FaceDatum   entry_face;                       // which face the screw head sits in
    double      position_x_mm  = 0.0;             // common axis center, world XY
    double      position_y_mm  = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };// drilling direction (into the material)
    double      pilot_dia_mm    = 0.0;            // cylindrical bore diameter
    double      pilot_depth_mm  = 0.0;            // measured from entry face
    double      cone_top_dia_mm = 0.0;            // large diameter at the entry face
    double      cone_angle_deg  = 90.0;           // INCLUDED angle (standard: 82° / 90° / 100° / 120°)
};

// Synthesis: apply this skill to a workpiece.
// Returns a new workpiece (cloned with the countersink cut) + the FeatureSignature.
SkillOutput apply(const Workpiece& wp, const Input& in);

// DFM validation.
DFMReport validate(const Workpiece& wp, const Input& in);

// Recognize candidates of this skill in the given workpiece.
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Derived geometry: cone_depth = (top_dia − pilot_dia) / 2 / tan(angle/2).
// Returns 0 if inputs are degenerate.
double computeConeDepth(const Input& in);

}  // namespace countersink
}  // namespace koocadcam::skill
