#pragma once
// @lat: [[engine/skills#lcd_bezel_window_compound]]
//
// lcd_bezel_window_compound — COMPOUND LCD display bezel recess +
// optional venting slots above the LCD area.
//
// Sub-feature chain (1 + optional 4 ops):
//   1. CUT rounded-corner rectangular recess for the LCD module (uses
//      Tools.hpp roundedRectPocketTool which is built on
//      BRepBuilderAPI_MakeWire-style rounded rectangles via fillet).
//   2. (optional) CUT N venting slots above the LCD recess, evenly
//      distributed across the LCD width.
//
// DFM gates:
//   DFM-INPUT      : window dims <= 0 or recess depth <= 0.
//   DFM-LCD-CORNER : lcd_window_corner_radius_mm < 0.5 (cutter floor).
//   DFM-LCD-CORNER-MAX: corner_radius > min(w,h)/2 (geometrically invalid).
//   DFM-LCD-VENTS  : vent_slot_count not in {0, 4} (this skill only supports
//                    either no vents or a 4-slot strip).

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lcd_bezel_window_compound {

constexpr const char* kSkillId = "lcd_bezel_window_compound";

struct Input
{
    int    face_id                       = -1;
    // LCD recess centre on the host face plane.
    double center_x_mm                   = 0.0;
    double center_y_mm                   = 0.0;
    double lcd_window_w_mm               = 0.0;
    double lcd_window_h_mm               = 0.0;
    double lcd_window_corner_radius_mm   = 0.0;
    double lcd_recess_depth_mm           = 0.0;
    // Either 0 (no vents) or 4 (a 4-slot top strip).
    int    vent_slot_count               = 0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lcd_bezel_window_compound
}  // namespace koocadcam::skill
