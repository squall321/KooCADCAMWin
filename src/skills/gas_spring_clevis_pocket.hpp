#pragma once
// @lat: [[engine/skills#gas_spring_clevis_pocket]]
//
// gas_spring_clevis_pocket — COMPOUND FEATURE (3 primitive sub-cuts).
//
// Realistic gas-spring U-clevis mounting pocket on a metal frame.  Used for
// attaching the rod-end of a hood-prop / hatch / industrial gas spring.
//
//   Sub-feature chain:
//     1. U-shape clevis pocket  (rectangular slot the width of the gas
//                                spring rod-end clevis), depth = clevis_d.
//     2. Cross-pin through-hole (M8 default — cylindrical hole through
//                                BOTH side walls of the pocket, axis
//                                perpendicular to pocket axis).
//     3. Retention groove       (small annular groove around the pin
//                                hole, snap-ring or e-clip seat).
//
//   The three cutters are NOT all concentric — pocket and pin hole share
//   the entry-face plane but pin hole axis is perpendicular.  We model
//   the pin hole as a separate cylinder cutter going through the part,
//   and groove as a thin annular ring on the pin axis.  All three are
//   compounded then a single cut() removes everything.
//
// Output topology: 2 planar side walls + 1 planar bottom + 1 cylindrical
// pin hole (through-both-walls) + 2 small annular grooves.  Single
// Workpiece, single FeatureSignature with pattern.is_compound = true,
// subfeature_count = 3.
//
// DFM gates (SAE J515 hydraulic-clevis pattern + DIN 71752):
//   - clevis_w >= 6 mm and <= 40 mm (covered Bansbach/Stabilus catalog)
//   - clevis_d >= clevis_w / 2 (pin needs side-wall material)
//   - pin_dia >= 4 mm (typical M4 minimum gas-spring eyelet)
//   - pin_dia < clevis_w / 2 (pin must fit between walls with margin)
//   - retention groove width = 1 mm fixed (e-clip standard)
//
// Recognize: metadata replay (compound clevis pocket has multi-axis
// topology that geometric fingerprinting can't disambiguate).
// Returns confidence 1.0 when matched from history.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace gas_spring_clevis_pocket {

constexpr const char* kSkillId = "gas_spring_clevis_pocket";

struct Input
{
    FaceDatum   face_id;
    double      position_x_mm  = 0.0;
    double      position_y_mm  = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };  // pocket cut direction
    double      clevis_w       = 0.0;               // pocket width (rod-end width)
    double      clevis_d       = 0.0;               // pocket depth into part
    double      clevis_length  = 0.0;               // pocket length (along part)
    double      pin_dia        = 8.0;               // cross-pin diameter
    double      retention_groove_w = 1.0;           // e-clip groove width
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace gas_spring_clevis_pocket
}  // namespace koocadcam::skill
