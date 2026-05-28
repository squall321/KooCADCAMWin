#pragma once
// @lat: [[engine/skills#laser_mark]]
//
// laser_mark — laser surface marking. A focused laser discolors / lightly
// ablates the surface without removing significant material.  Geometrically
// this is a VERY SHALLOW engrave (5 – 200 µm).
//
//                   entry_face (planar)
//                         │
//        ─────────────────┼──────────────────────
//           ░ ░ ░ ░ ░     │                   <- character glyph footprint
//                         │                      (each char ~ µm deep)
//                         │
//
// Signature pattern:
//   - In the FULL path: a chain of font-glyph polygon extrusions on the
//     entry face, depth = mark_depth_um × 1e-3 mm (sub-tolerance for
//     typical OCCT precision Confusion = 1e-7 m = 1e-4 mm, but well inside
//     the BRepAlgo robustness band for marks ≥ ~5 µm).
//   - For very shallow marks (depth < 0.001 mm == 1 µm), the Boolean cut
//     would collapse — we fall back to per-character SPOT CYLINDERS (one
//     small cylinder centred on each char's glyph centre) so the result
//     remains a valid solid distinguishable from the input.
//
// DFM checks:
//   DFM-LASER-DEPTH  : mark_depth_um ∈ [5, 200]
//   DFM-LASER-FONT   : font_size_mm ≥ 0.5 (laser-min readable)
//
// Recognize:
//   Metadata replay is the primary path (sub-mm features near OCCT
//   tolerance are difficult to fingerprint geometrically).  Heuristic
//   fallback: row of very-shallow cylindrical pits on a planar face.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace laser_mark {

constexpr const char* kSkillId = "laser_mark";

struct Input
{
    FaceDatum   entry_face;                       // planar face being marked
    double      position_x_mm = 0.0;              // baseline start, world XY
    double      position_y_mm = 0.0;
    gp_Dir      direction_xy  { 1.0, 0.0, 0.0 };  // text-baseline direction
    std::string text          = "";
    double      font_size_mm  = 2.0;
    double      mark_depth_um = 30.0;             // micrometres (default 30 µm)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace laser_mark
}  // namespace koocadcam::skill
