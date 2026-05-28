#pragma once
// @lat: [[engine/skills#stretching]]
//
// stretching — pull material at a localized region to make it thinner and
// longer.  Modelled as a shallow concave depression (inverted emboss) at
// (position_x, position_y), of radius `stretch_radius_mm` and depth equal
// to (sheet_thickness × target_thickness_reduction_pct / 100).
//
//                  ──────────────.───────.──────────
//                                  ──.──
//                                stretched zone
//                                (thinner, depressed)
//
// Synthesis strategy:
//   Cut a shallow cylindrical depression from the sheet's top face:
//     - cylinder of radius `stretch_radius_mm`,
//     - depth = sheet_thickness × (reduction_pct/100)
//   Like `emboss` but inverted (depression instead of bump).
//
// Signature pattern:
//   - 1 cylindrical face (depression wall, vertical axis)
//   - 1 small planar disc (depression bottom) — a circular flat below the
//     sheet plane
//   - shape signature recorded so it can be distinguished from emboss
//
// DFM checks:
//   DFM-ST-REDUCTION : target_thickness_reduction_pct ≤ 30
//   DFM-ST-RADIUS    : stretch_radius_mm > 5 × sheet_thickness
//   DFM-ST-SHEET     : sheet thickness ≤ 5 mm
//   DFM-INPUT        : stretch_radius_mm > 0, reduction_pct > 0
//
// Recognize:
//   Vertical-axis cylindrical face that DESCENDS into the sheet top
//   (bottom-of-feature Z < sheet top Z), bounded by a small planar disc.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace stretching {

constexpr const char* kSkillId = "stretching";

struct Input
{
    FaceDatum   entry_face;                            // sheet top
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    double      stretch_radius_mm = 0.0;
    double      target_thickness_reduction_pct = 0.0;  // [0..30]
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Helper — sheet thickness from bbox.
double sheetThickness(const Workpiece& wp);

}  // namespace stretching
}  // namespace koocadcam::skill
