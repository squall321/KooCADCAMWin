#pragma once
// @lat: [[engine/skills#deep_draw]]
//
// deep_draw — pull a flat sheet into a cup or cylindrical shape using a
// punch + die.  Sheet metal forming primary operation.
//
//                           ↑ cup_depth_mm
//                           │
//                  ┌─────┐  │
//                  │     │  │  ← cylindrical wall (sheet_thickness)
//                  │     │  │
//                  │     │  │
//                  └─────┘  ↓
//                     │
//                     └── circular bottom (rounded with corner_r_mm)
//                  ──────────────────────────  ← original sheet plane
//                       (cup at cup_position)
//
// Synthesis strategy:
//   1. cut a circular through-hole of cup_dia at (cup_position_x, cup_position_y).
//   2. fuse a hollow cylindrical cup back into the hole:
//        - outer cylinder of radius = cup_dia/2, height = cup_depth
//        - inner cylinder of radius = (cup_dia/2 - sheet_thickness)
//          subtracted from the outer to form a wall of thickness = sheet_thickness
//   3. the bottom of the cup is sealed (flat disc at cup_depth below the
//      sheet plane), with a fillet of corner_r_mm where the wall meets it.
//
// Signature pattern:
//   - 1 cylindrical face (cup wall, axis = +Z)
//   - 1 planar disc (cup bottom)
//   - 1 toroidal/fillet face (wall→bottom corner, radius = corner_r_mm)
//   - sheet plane with a circular hole the cup protrudes through
//
// DFM checks:
//   DFM-DD-RATIO   : drawing_ratio = cup_depth / cup_dia ≤ 0.7
//                    (single-pass deep draw max — beyond this, redraw needed)
//   DFM-DD-CORNER  : corner_r ≥ 4 × sheet_thickness
//                    (sharp corners tear in deep draws)
//   DFM-DD-SHEET   : sheet thickness ≤ 5 mm
//   DFM-INPUT      : cup_dia > 0, cup_depth > 0
//
// Recognize:
//   A cylindrical protrusion from a thin planar sheet with depth > thickness.
//   Find a vertical-axis cylinder whose axial span > sheet_thickness, bounded
//   at the bottom by a small circular disc (cup bottom).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace deep_draw {

constexpr const char* kSkillId = "deep_draw";

struct Input
{
    FaceDatum   entry_face;                 // sheet top
    double      cup_position_x_mm = 0.0;    // cup centre in world XY
    double      cup_position_y_mm = 0.0;
    double      cup_dia_mm        = 0.0;    // cup outer diameter
    double      cup_depth_mm      = 0.0;    // depth below sheet plane
    double      corner_r_mm       = 0.0;    // bottom corner fillet
    double      sheet_thickness_mm = 0.0;   // 0 → auto-detect from bbox
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Helper — return the smallest bbox extent (= sheet thickness).
double sheetThickness(const Workpiece& wp);

}  // namespace deep_draw
}  // namespace koocadcam::skill
