#pragma once
// @lat: [[engine/skills#sheet_punch]]
//
// sheet_punch — punch a through hole in sheet metal stock.
//
// Topologically identical to a through drill_hole, but tagged with the
// sheet-metal tooling family (punch press, not twist drill) and gated by
// the sheet-metal punch-min-ratio DFM rule (dia ≥ 1.5 × thickness).
//
//                       entry_face (sheet top)
//                              │
//                  ────────────┼────────  ↑
//                              │          │ sheet_thickness (≤ 5 mm)
//                              │   ↓ axis │
//                  ────────────┴────────  ↓
//                              ↑
//                          dia_mm  (≥ 1.5 × thickness)
//
// Signature pattern:
//   - 1 cylindrical face spanning the full sheet thickness
//   - 2 circular edges (one on each side of the sheet)
//   - NO bottom planar face (always through)
//
// DFM checks:
//   DFM-SP-MIN-DIA : dia ≥ 1.5 × sheet_thickness
//   DFM-SP-SHEET   : sheet_thickness ≤ 5 mm (one bbox extent must be < 5)
//   DFM-INPUT      : dia > 0
//
// Recognize:
//   Find a cylindrical face whose axial extent equals the smallest bbox
//   extent of the workpiece (the sheet thickness); both bounding circular
//   edges sit on opposite planar faces of the sheet (top/bottom).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sheet_punch {

constexpr const char* kSkillId = "sheet_punch";

struct Input
{
    FaceDatum   entry_face;             // typically the +Z sheet top
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    double      dia_mm        = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };  // punch direction
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Helper — return the smallest bbox extent (= sheet thickness for a
// cuboid sheet stock).  Used by both validate() and recognize().
double sheetThickness(const Workpiece& wp);

}  // namespace sheet_punch
}  // namespace koocadcam::skill
