#pragma once
// @lat: [[engine/skills#bore_cylindrical]]
//
// bore_cylindrical — precision cylindrical bore (tight-tolerance hole).
//
// Geometrically identical to drill_hole's blind / through hole, but the
// process and metadata differ:
//   - Tool: single-point boring bar (not a twist drill).
//   - Feed: slower; SFM ≈ 100 (vs ≈ 300 for drilling aluminum).
//   - Tolerance: H7/H8/H9 ground-finish class.
//   - Diameter range: typically 6 mm and up (smaller dia => use drill+ream).
//
//                          ┌─ entry_face (planar)
//                          ▼
//                    ──────┬───────────┬──────
//                          │   ▼ axis  │
//                          │           │
//                          │           │       ↑ depth_mm
//                          │           │       │
//                          │           │       │
//                          └───────────┘       ↓
//                                ↑
//                            diameter_mm
//
// Signature pattern:
//   - 1 cylindrical face (the bore wall)
//   - 1 circular edge at the entry face (top)
//   - 1 circular edge at the bottom (where wall meets bottom)
//   - 1 planar face (blind bottom)
//   - signature.pattern.kind == "bore_cylindrical"
//   - signature.pattern.tolerance_class carries the precision grade.
//
// DFM checks:
//   DFM-002      : diameter_mm ≥ 0.8 mm
//   boring-bar  : diameter_mm ≥ 6.0 mm recommended (warning if smaller)
//   boring-bar  : depth/dia ≤ 4.0      (warning above — bar deflection)
//
// Recognize:
//   Same topology as drill_hole.  Differentiated by recovered metrics:
//   "bore" identity requires diameter ≥ 6 mm AND depth/diameter ≤ 4.
//   When both criteria match cleanly, confidence ≈ 0.9; when only one
//   matches, confidence drops (ambiguous with drill_hole).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bore_cylindrical {

constexpr const char* kSkillId = "bore_cylindrical";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm  = 0.0;
    double      position_y_mm  = 0.0;
    double      position_z_mm  = 0.0;     // entry point Z (0 => snap to ±Z face)
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    double      diameter_mm    = 0.0;
    double      depth_mm       = 0.0;
    std::string tolerance_class = "H7";   // "H7" | "H8" | "H9"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bore_cylindrical
}  // namespace koocadcam::skill
