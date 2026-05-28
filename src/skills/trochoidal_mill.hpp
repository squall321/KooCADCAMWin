#pragma once
// @lat: [[engine/skills#trochoidal_mill]]
//
// trochoidal_mill — high-feed pocket-roughing strategy.
//
// The end-mill moves in small circular loops (trochoids) while progressing
// along the pocket footprint.  Radial engagement stays low (≈ stepover) so
// the cutter can run at much higher feed and surface speed than a
// conventional rectangular pocket strategy.  Chip evacuation is also better
// because each trochoid loop exits the cut.
//
//                       entry_face (planar)
//                              │
//                  ────────────┼──────────────
//                              │
//                   ┌──────────┴──────────┐    ↑ depth_mm
//                   │ ○ ○ ○ ○ ○ ○ ○ ○     │    │   (trochoid loops; geom
//                   │ ○ ○ ○ ○ ○ ○ ○ ○     │    │    result identical to a
//                   │ ○ ○ ○ ○ ○ ○ ○ ○     │    │    flat rect pocket)
//                   └─────────────────────┘    ↓
//                          length_mm
//
// Geometrically the result is identical to a `mill_rect_pocket`; the
// difference lives in the tooling metadata + `signature.pattern.kind`:
//   - tool_type = "end_mill_high_feed"
//   - cutting_speed_sfm × 1.5 vs conventional pocket
//   - feed_per_tooth_mm × 2 vs conventional pocket
//   - stepover_mm recorded for CAM toolpath generation
//
// Signature pattern: rect pocket topology (4 walls + 4 corner cyls +
// bottom) with `pattern.kind == "trochoidal_mill"`.
//
// DFM checks:
//   DFM-002      : tool_dia_mm ≥ 1.0 (high-feed strategy needs slightly
//                  larger tool than the absolute end-mill min for stiffness)
//   DFM-TROCH-SO : stepover_mm ≤ tool_dia × 0.3 (any larger and the
//                  trochoidal advantage is lost — it becomes conventional)
//
// Recognize:
//   Geometric topology overlaps with `mill_rect_pocket`.  We claim a
//   recognition only when the workpiece carries a previously-stamped
//   trochoidal feature signature; otherwise plain rect pocket should win.
//   For the in-memory case we deliver confidence 0.40 (marker that the
//   recognizer leaves to higher-confidence rect-pocket recognition).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace trochoidal_mill {

constexpr const char* kSkillId = "trochoidal_mill";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm   = 0.0;
    double      center_y_mm   = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      length_mm     = 0.0;
    double      width_mm      = 0.0;
    double      depth_mm      = 0.0;
    double      corner_r_mm   = 1.0;
    double      tool_dia_mm   = 6.0;
    // Trochoid advance per loop.  Default = tool_dia × 0.1.
    // 0.0 → auto-fill at apply-time.
    double      stepover_mm   = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace trochoidal_mill
}  // namespace koocadcam::skill
