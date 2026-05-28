#pragma once
// @lat: [[engine/skills#dot_peen_mark]]
//
// dot_peen_mark — a pneumatic / electric stylus indents a DOT PATTERN into
// the workpiece surface, one tiny cylindrical pit per dot.  The dots are
// arranged in glyph shapes so the pattern reads as text.
//
//                   entry_face (planar)
//                         │
//        ─────────────────┼──────────────────────
//            ▪ ▪ ▪         ▪ ▪ ▪
//            ▪   ▪         ▪      ▪          ← per-character dot grid
//            ▪ ▪ ▪         ▪      ▪
//            ▪             ▪      ▪
//            ▪             ▪ ▪ ▪
//
// Each character is rendered by sampling a fixed 5×7 GRID over the glyph's
// UNIT bounding box: at every grid cell whose UV centre lies INSIDE the
// glyph polygon (from `_glyph_table.hpp`), we drill a tiny cylinder of
// (dot_dia × dot_depth).  Characters not in the glyph table fall back to
// an empty/solid block — see `apply()` for the exact rule.
//
// Signature pattern:
//   - pattern.kind = "dot_peen_mark"
//   - text + font_size + dot_dia + dot_depth + dot_spacing
//   - geometry = "dot_matrix"
//
// DFM checks:
//   DFM-PEEN-DIA   : dot_dia ≥ 0.1
//   DFM-PEEN-DEPTH : dot_depth ≥ 0.05
//   DFM-INPUT      : text non-empty, font_size > 0
//
// Recognize:
//   Metadata replay only.  Geometric recognition of an N-dot pattern
//   embedded in glyph shapes would require dot-cluster classification —
//   out of scope.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace dot_peen_mark {

constexpr const char* kSkillId = "dot_peen_mark";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm  = 0.0;
    double      position_y_mm  = 0.0;
    gp_Dir      direction_xy   { 1.0, 0.0, 0.0 };
    std::string text           = "";
    double      font_size_mm   = 3.0;
    double      dot_dia_mm     = 0.3;   // pit diameter
    double      dot_depth_mm   = 0.2;   // pit depth
    double      dot_spacing_mm = 0.4;   // centre-to-centre spacing in dot matrix
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace dot_peen_mark
}  // namespace koocadcam::skill
