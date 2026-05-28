#pragma once
// @lat: [[engine/skills#electrochemical_etch]]
//
// electrochemical_etch — electrolyte + stencil + low DC voltage etches a
// pattern into a conductive metal surface.  Used heavily on stainless
// steel & high-strength alloys where laser oxidation is undesirable.
//
//                   entry_face (planar)
//                         │
//        ─────────────────┼──────────────────────
//           ▓ ▓ ▓ ▓ ▓     │              <- chemically eroded glyph
//                         │                 (uniform shallow depression)
//                         │
//
// Process limits (vs laser_mark):
//   - Stencil-driven: minimum font size ~ 1 mm (smaller stencils blur)
//   - Depth range: 10–200 µm (deeper requires multi-pass agitation)
//
// Signature pattern:
//   - pattern.kind = "electrochemical_etch"
//   - geometry = glyph-shaped depression at sub-mm depth
//
// DFM checks:
//   DFM-ETCH-DEPTH : etch_depth_um ∈ [10, 200]
//   DFM-ETCH-FONT  : font_size_mm ≥ 1.0 (stencil limitation)
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace electrochemical_etch {

constexpr const char* kSkillId = "electrochemical_etch";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      direction_xy  { 1.0, 0.0, 0.0 };
    std::string text          = "";
    double      font_size_mm  = 3.0;
    double      etch_depth_um = 50.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace electrochemical_etch
}  // namespace koocadcam::skill
