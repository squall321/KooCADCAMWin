#pragma once
// @lat: [[engine/skills#wood_planing]]
//
// wood_planing — surface planing of a wooden board to flatten and reduce
// thickness on a planer / thicknesser.  The board top face is shaved by
// `removal_mm` per pass, producing a smooth flat surface and a thinner
// board.
//
//        ┌────────────────────────┐
//        │ rough/raised grain     │   ── pass 1 ──►   ┌────────────────────┐
//        │     ▒▒  ▒▒▒  ▒  ▒▒    │                   │ ░░ smooth surface  │
//        └────────────────────────┘                   └────────────────────┘
//         t = before                                   t = before - removal
//
// Synthesis strategy:
//   Same rebuild approach as rolling/sawmill_cut.  We shrink the workpiece
//   along its thickness axis (smallest bbox extent) by `removal_mm`.  The
//   resulting workpiece has the same XY footprint and 6 planar faces.
//
// Signature pattern:
//   - 6 planar faces (planed cuboid)
//   - thickness_axis     = "x" | "y" | "z"
//   - removal_mm         = depth shaved
//   - surface_finish     = "smooth" (post-planing Ra ≈ 6–15 µm wood scale)
//   - target_thickness_mm = after-pass thickness
//
// DFM checks:
//   DFM-PLANE-PASS    removal_mm > 3.0 mm  → error (tear-out + motor overload)
//   DFM-PLANE-MIN     removal_mm < 0.2 mm  → warn  (scraping, not cutting)
//   DFM-PLANE-LEAVE   target_thickness_mm < 6 mm → error (board too thin)
//   DFM-INPUT         removal_mm <= 0
//
// Recognize:
//   Geometric recognition cannot determine if a board was planed — confidence
//   is high only when metadata is present.  The fallback geometric candidate
//   simply reports the current thickness axis.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace wood_planing {

constexpr const char* kSkillId = "wood_planing";

struct Input
{
    FaceDatum   entry_face;             // face being planed (top)
    double      removal_mm    = 0.5;    // depth removed per pass
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Helper — return the smallest bbox extent (the inferred plane axis).
double currentThickness(const Workpiece& wp);

}  // namespace wood_planing
}  // namespace koocadcam::skill
