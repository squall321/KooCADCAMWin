#pragma once
// @lat: [[engine/skills#bore_and_finish]]
//
// bore_and_finish — COMPOUND MACRO for a precision hole:
//   drill (rough) → bore (semi-finish) → ream/final-bore (finish).
//
// The end result is a tight-tolerance cylindrical bore at exactly
// `final_diameter_mm`.  The macro chains three (or two — see fallback)
// material-removal sub-skills so each pass removes a controlled depth-of-cut.
//
//                          ┌─ entry_face (planar)
//                          ▼
//                    ──────┬──────────┬──────
//                          │  ▼ axis  │
//                          │          │       ↑ depth_mm
//                          │          │       │
//                          │          │       │
//                          └──────────┘       ↓
//                                ↑
//                         final_diameter_mm   (with H7 tolerance)
//
// Sub-skill sequence:
//   1. drill_hole       : dia = final - 1.5 mm  (rough opening, clears bulk)
//   2. bore_cylindrical : dia = final - 0.2 mm  (semi-finish, leaves 0.1 mm
//                                                radial finishing allowance)
//   3. ream             : dia = final          (FINISH — currently NOT
//                                                IMPLEMENTED as a separate
//                                                skill; we use a tiny final
//                                                bore_cylindrical call to
//                                                bring the bore to size.)
//
// Fallback note:
//   This commit assumes `ream` skill is NOT yet available.  We model the
//   third step as a final tight bore_cylindrical pass at exact diameter.
//   When the ream skill lands (alpha agent's output), replace the marked
//   block with `ream::apply` and keep the same SkillOutput shape.
//
// Signature pattern:
//   - 1 cylindrical face at the FINAL diameter (the only one visible after
//     all 3 passes).
//   - 1 circular edge at the entry face, 1 at the bottom.
//   - 1 planar bottom face.
//   - signature.pattern.kind == "bore_and_finish"
//   - signature.pattern.tolerance_class records the requested precision.
//
// DFM:
//   final_diameter_mm ≥ 1.5 mm (need margin for the multi-step process —
//   drill at final - 1.5 must still produce a valid hole of dia ≥ 0.0).
//   depth/dia ≤ 4.0  (inherited from bore_cylindrical bore-bar deflection
//   limit; warning).
//
// Recognize:
//   A single cylindrical face with depth/dia ≤ 4 AND diameter ≥ 1.5 mm
//   may be a finished bore.  Confidence ≈ 0.7 (overlaps with
//   bore_cylindrical and drill_hole).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bore_and_finish {

constexpr const char* kSkillId = "bore_and_finish";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm    = 0.0;
    double      position_y_mm    = 0.0;
    gp_Dir      axis_dir         { 0.0, 0.0, -1.0 };
    double      final_diameter_mm = 0.0;
    double      depth_mm          = 0.0;
    std::string tolerance_class   = "H7";   // H7 default for precision bore
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bore_and_finish
}  // namespace koocadcam::skill
