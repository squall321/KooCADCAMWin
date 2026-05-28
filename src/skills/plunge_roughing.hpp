#pragma once
// @lat: [[engine/skills#plunge_roughing]]
//
// plunge_roughing — pocket roughing by a GRID of repeated plunge drills.
//
// Unlike `plunge_milling` (a single hole), plunge_roughing fills a
// rectangular pocket footprint with many plunge points spaced by
// `plunge_stepover_mm`.  Between plunges, the tool retracts and traverses
// to the next XY location, then plunges again.  Once the grid is complete
// a final lateral cleanup pass removes the small "scallops" left between
// adjacent plunges.
//
//                       entry_face (planar)
//                              │
//                  ────────────┼──────────────
//                              │
//                   ┌──────────┴──────────┐    ↑ depth_mm
//                   │ ▽ ▽ ▽ ▽ ▽ ▽ ▽ ▽     │    │  ▽ = plunge points;
//                   │ ▽ ▽ ▽ ▽ ▽ ▽ ▽ ▽     │    │      lateral cleanup pass
//                   │ ▽ ▽ ▽ ▽ ▽ ▽ ▽ ▽     │    │      finalizes the floor
//                   └─────────────────────┘    ↓
//                          length_mm
//
// Geometrically — at slice-1 fidelity — the result is identical to a
// `mill_rect_pocket`: the cleanup pass produces the same flat-bottom
// pocket.  The strategy difference (and the reason a CAM engineer would
// pick plunge_roughing) is reflected in tooling metadata:
//   - tool_type = "end_mill_plunge_capable"
//   - reduced cutting_speed_sfm (plunge cutting is slower)
//   - reduced feed_per_tooth_mm
//   - signature records the plunge grid for CAM toolpath generation
//
// DFM checks:
//   DFM-002          : tool_dia_mm ≥ 0.8 mm
//   DFM-PLUNGE-STEP  : plunge_stepover_mm ≤ tool_dia_mm
//                       (above this, the plunges leave un-removed material
//                        beyond the cleanup-pass reach)
//
// Recognize:
//   Topologically the same as `mill_rect_pocket`.  Use the same dual-path
//   strategy as trochoidal_mill / adaptive_clearing: feature-history first
//   (conf 0.90), geometric marker fallback (conf 0.40).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace plunge_roughing {

constexpr const char* kSkillId = "plunge_roughing";

struct Input
{
    FaceDatum   entry_face;
    double      center_x_mm         = 0.0;
    double      center_y_mm         = 0.0;
    gp_Dir      axis_dir            { 0.0, 0.0, -1.0 };
    double      length_mm           = 0.0;
    double      width_mm            = 0.0;
    double      depth_mm            = 0.0;
    double      corner_r_mm         = 1.0;
    double      tool_dia_mm         = 6.0;
    // XY distance between plunge centers.  Default tool_dia × 0.6.
    // 0 → auto-fill at apply-time.
    double      plunge_stepover_mm  = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace plunge_roughing
}  // namespace koocadcam::skill
