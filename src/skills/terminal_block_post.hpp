#pragma once
// @lat: [[engine/skills#terminal_block_post]]
//
// terminal_block_post — protruding screw-terminal post on an insulator base.
// Combines ADD operations (fuse) and SUBTRACT operations (cut) to build:
//   1. Threaded stud cylinder (protrudes up from base).
//   2. Retention shoulder (wider disc at the stud's base — keeps it from
//      pulling out of the insulator).
//   3. Insulation skirt (taller, larger-diameter cylinder around the stud
//      base — provides creepage distance per IEC 60664).
//   4. Two wire holes through the skirt (perpendicular to the stud axis,
//      diametrically opposed) — for the wire to be clamped under the screw.
//
// Sub-feature count: 4 (stud + shoulder + skirt + 2 wire holes counted as
// 1 "wire-hole-pair" subfeature, total = 4).
//
//                       │ stud_dia
//                  ┌────┴────┐                ↑ stud_height
//                  │         │                │
//                  │         │                ↓
//                ┌─┴─────────┴─┐    ← shoulder ↑ shoulder_height
//                └─┬─────────┬─┘                ↓
//                  │         │                ↑
//                  │   ▒▒▒   │ ← wire holes   │ skirt_height
//                  │   ▒▒▒   │                │
//                  └─────────┘                ↓
//                  ◄─skirt_dia─►
//
// DFM (IEC 60947 / UL 1059):
//   - stud_dia_mm ≥ 2 mm     (M2 stud min)
//   - skirt_dia_mm > shoulder_dia_mm > stud_dia_mm
//   - wire_hole_dia_mm ≥ 1 mm
//   - skirt_height_mm ≥ 5 mm (creepage distance ≈ 5 mm for 50V/CAT II)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace terminal_block_post {

constexpr const char* kSkillId = "terminal_block_post";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm     = 0.0;   // post center, world XY
    double      position_y_mm     = 0.0;
    gp_Dir      axis_dir          { 0.0, 0.0, 1.0 };  // upward (additive)
    double      stud_dia_mm       = 3.0;   // M3 thread blank
    double      stud_height_mm    = 8.0;
    double      shoulder_dia_mm   = 5.0;
    double      shoulder_height_mm = 1.5;
    double      skirt_dia_mm      = 8.0;
    double      skirt_height_mm   = 5.0;   // IEC 60664 creepage
    double      wire_hole_dia_mm  = 1.6;   // accepts AWG 16
    double      wire_hole_z_mm    = 2.5;   // height of hole center above the base
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace terminal_block_post
}  // namespace koocadcam::skill
