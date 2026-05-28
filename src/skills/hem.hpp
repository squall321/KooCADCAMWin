#pragma once
// @lat: [[engine/skills#hem]]
//
// hem — a 180° fold (fold the sheet edge back on itself).  Used for
// safety edges, increased stiffness, or as a decorative finish.
//
//                   ┌──── hem (folded back, parallel to sheet)
//                   ▼
//                  ▕▔▔▔▔▔▔▔▔▏  ← gap = 0 (flat_hem=true) or > 0 (open)
//                   ─────────────────────  ← sheet
//                                                 │ thickness
//                   ─────────────────────
//
// For slice 1 the synthesis is approximate: we add a thin extra layer
// of material on top of the sheet along the chosen edge, of width
// `hem_width_mm`.  The bottom face of this extra layer sits exactly at
// (sheet top + gap), where gap = 0 if `flat_hem=true` and equals the
// sheet thickness if `flat_hem=false` (open hem).
//
// Signature pattern:
//   - 1 extra parallel-planar face (the top of the hem) offset above
//     the sheet's top face by (sheet_thickness × 2 + gap) when flat_hem,
//     or a separated layer when open
//   - 1 cylindrical bend face wrapping at the edge (axis along the edge)
//
// DFM checks:
//   DFM-H-MIN-W  : hem_width_mm ≥ 2 × sheet_thickness
//   DFM-H-SHEET  : sheet thickness ≤ 5 mm
//
// Recognize:
//   Parallel-planar pair separated by ~ sheet_thickness along the sheet
//   edge.  The upper face's area ≈ hem_width × edge_length.

#include "Datum.hpp"
#include "Skill.hpp"
#include "flange.hpp"     // reuse EdgeSide enum

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hem {

constexpr const char* kSkillId = "hem";

using flange::EdgeSide;
using flange::edgeSideToString;
using flange::edgeSideFromString;

struct Input
{
    EdgeSide  edge_selector = EdgeSide::XMax;
    double    hem_width_mm  = 4.0;
    bool      flat_hem      = true;   // true = closed flat, false = open (gap = thickness)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hem
}  // namespace koocadcam::skill
