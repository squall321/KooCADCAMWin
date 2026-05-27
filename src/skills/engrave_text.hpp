#pragma once
// @lat: [[engine/skills#engrave_text]]
//
// engrave_text — V-cut engraving of text characters into a planar face.
// Typical depth 0.1–0.3 mm with a small V-cutter or end-mill.
//
//                   entry_face (planar)
//                         │
//        ─────────────────┼──────────────────────
//           ● ● ● ● ●     │                   <- character spots
//                         │                      (one per char in `text`)
//                         │
//
// Signature pattern:
//   - In the FULL implementation: a chain of font-glyph wire extrusions
//     forming letter shapes on the entry face.
//   - In this APPROXIMATION (slice 2): N small cylindrical spot cuts, one
//     per character, spaced by font_size_mm along +X.  Diameter equals
//     stroke_width_mm, depth equals depth_mm.  This produces a valid solid
//     that captures the engrave INTENT (small shallow cylindrical marks in
//     a row) without doing real font rasterization.
//
// DFM checks:
//   DFM-019 : stroke_width_mm ≥ 0.15 mm (min V-cutter stroke)
//   depth_mm ∈ [0.05, 0.5]
//
// Recognize:
//   Find shallow (depth < 0.5 mm) cylindrical features with small
//   diameter (< 1 mm) arranged in a row.  Confidence low without
//   metadata; if a prior signature exists in workpiece.features(),
//   replay it with full confidence.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace engrave_text {

constexpr const char* kSkillId = "engrave_text";

struct Input
{
    FaceDatum   entry_face;             // planar face being engraved
    double      position_x_mm   = 0.0;  // baseline start, world XY
    double      position_y_mm   = 0.0;
    gp_Dir      direction       { 1.0, 0.0, 0.0 };  // text-baseline direction
    std::string text            = "";
    double      font_size_mm    = 2.0;  // character pitch (spot spacing)
    double      stroke_width_mm = 0.2;  // spot diameter (= tool diameter)
    double      depth_mm        = 0.15;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace engrave_text
}  // namespace koocadcam::skill
