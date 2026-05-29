#pragma once
// @lat: [[engine/skills#din_rail_clip_slot]]
//
// din_rail_clip_slot — COMPOUND ENCLOSURE FEATURE that machines a DIN-rail
// (EN 60715, 35 × 7.5 mm "TS35") mounting cutout into the back face of an
// enclosure.  Three primitive cuts fused into one signature:
//
//   1. Top rail groove   — rectangular slot, 27.5 mm wide × 5.5 mm deep
//                          (the hat-rail body sits in here).
//   2. Bottom hook ledge — narrower rectangular slot offset 17 mm below the
//                          rail centerline that catches the spring-loaded
//                          retention clip.
//   3. Retention chamfer — 0.8 × 45° break on the upper rail edge so the rail
//                          slides in without snagging the gasket.
//
// All three cuts share the same axis_dir (into the face) and rail_dir_deg
// (rotation around the face normal).
//
//                rail_length_mm
//   ──────────────────────────────────────────  ← chamfer (cut #3)
//   ┌────────────────────────────────────────┐
//   │                                        │
//   │           top rail groove (#1)         │   27.5 mm
//   │                                        │
//   └────────────────────────────────────────┘
//                    │   17 mm
//                    │
//                ┌───┴──┐
//                │ #2   │   hook ledge (5 mm wide × 2 mm deep)
//                └──────┘
//
// DFM checks:
//   DFM-INPUT          : rail_length_mm in [40, 500] mm
//   DFM-DIN-GROOVE     : groove walls span the full TS35 height (≥ 35 mm in
//                        rail-length direction × 27.5 mm wide tolerance)
//   DFM-DIN-WALL       : enclosure_face thickness must be ≥ 8 mm
//                        (groove + chamfer + 1.5 mm minimum back-wall)
//   DFM-002            : retention chamfer ≥ 0.5 mm (sub-min break fails)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace din_rail_clip_slot {

constexpr const char* kSkillId = "din_rail_clip_slot";

struct Input
{
    FaceDatum   enclosure_face;
    double      center_x_mm     = 0.0;   // XY centre of the rail on the face
    double      center_y_mm     = 0.0;
    gp_Dir      axis_dir        { 0.0, 0.0, -1.0 };  // into the back face
    double      rail_dir_deg    = 0.0;   // rail long-axis rotation (CCW from +X)
    double      rail_length_mm  = 100.0; // rail-direction span
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace din_rail_clip_slot
}  // namespace koocadcam::skill
