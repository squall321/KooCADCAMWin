#pragma once
// @lat: [[engine/skills#tilt_post_for_lcd_panel]]
//
// tilt_post_for_lcd_panel — Smart-electronics compound skill that adds a
// tilt mechanism for an LCD panel, consisting of:
//   1. PIVOT-PIN POCKET  — material REMOVED (cut): blind cylindrical hole
//                          (the pivot pin's bearing seat).
//   2. BACK SUPPORT PILLAR — material ADDED (fuse): rectangular pillar that
//                          rises behind the pivot pocket; auto-EXTENDS the
//                          base plate up to the back to ensure the pillar
//                          attaches to solid material.
//   3. C-CLIP RETENTION GROOVE — narrow annular groove cut INSIDE the
//                          pivot pocket so a snap-ring (DIN 471) holds the
//                          pin axially.
//
// CONTEXT-AWARE:
//   - Queries wp.bbox() to discover the existing plate footprint.
//   - If the pivot site sits within margin of the +Y back edge of the
//     plate (or beyond it), the back-support pillar is fused with a small
//     extension plate so the pillar always lands on solid material.
//
// SPEC-TABLE DRIVEN by panel sizes:
//   "PANEL-7"  → pin Ø 3 mm, pocket 6 mm deep, pillar 8 × 6 × 12 mm
//   "PANEL-10" → pin Ø 4 mm, pocket 8 mm deep, pillar 10 × 8 × 16 mm
//   "PANEL-13" → pin Ø 5 mm, pocket 10 mm deep, pillar 12 × 10 × 20 mm
//   "PANEL-15" → pin Ø 6 mm, pocket 12 mm deep, pillar 14 × 12 × 24 mm
//   "PANEL-17" → pin Ø 8 mm, pocket 14 mm deep, pillar 16 × 14 × 28 mm
//
// DFM:
//   DIN 471         — circlip groove width per nominal shaft Ø
//   DFM-002         — pin Ø ≥ 0.8 mm
//   DFM-PILLAR-WALL — pillar wall ≥ 2 mm (pin Ø + 2·wall)
//   DFM-POCKET-WALL — pocket wall to bbox edge ≥ 1.5 mm

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>

namespace koocadcam::skill {

class Workpiece;

namespace tilt_post_for_lcd_panel {

constexpr const char* kSkillId = "tilt_post_for_lcd_panel";

struct Input
{
    FaceDatum    base_face;                       // top face of the plate
    gp_Dir       axis_dir            { 0.0, 0.0, -1.0 };
    double       position_x_mm       = 0.0;
    double       position_y_mm       = 0.0;
    std::string  panel_size          = "PANEL-10";  // see spec table
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tilt_post_for_lcd_panel
}  // namespace koocadcam::skill
