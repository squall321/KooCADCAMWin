#pragma once
// @lat: [[engine/skills#fiber_draw]]
//
// fiber_draw — draw a hair-thin glass fiber from a preform rod on a draw
// tower.  Preform is fed downward through a graphite furnace at 2000+ °C;
// the molten tip is pulled into a continuous fiber at controlled tension.
// The fiber diameter is locked by the ratio between feed-rate (preform) and
// pull-rate (fiber).
//
//          preform (Ø ~125 mm)
//              │
//              ▼
//        ┌──────────┐
//        │ furnace  │  ≈ 2050 °C
//        │  ▼▼▼     │
//        └────┬─────┘
//             │
//             ▼  (necked glass)
//             ◊
//             │  ← fiber Ø 125 µm
//             ▼
//        ┌──────────┐
//        │ coating  │  ← UV-cure acrylate
//        └──────────┘
//             │
//        ┌──────────┐
//        │  spool   │  draw_speed_m_min: 100–2000
//        └──────────┘
//
// Signature pattern:
//   - pattern.kind             = "fiber_draw"
//   - pattern.final_dia_um     = <input>
//   - pattern.draw_temp_c      = <input>
//   - pattern.draw_speed_m_min = <input>
//   - pattern.coating_applied  = bool (true if draw_speed reasonable for inline coater)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT             final_dia_um ≤ 0 OR draw_speed_m_min ≤ 0   (error)
//   DFM-DRAW-DIA          final_dia_um ∉ [50, 1000]                   (error)
//   DFM-DRAW-TEMP-LOW     draw_temp_c < 1900                          (error)
//   DFM-DRAW-TEMP-HIGH    draw_temp_c > 2200                          (error)
//   DFM-DRAW-SPEED        draw_speed_m_min ∉ [10, 3000]               (info)
//
// Synthesis:
//   Records the draw parameters in the workpiece feature list — no
//   topological change to the workpiece (this is a process-history stamp).
//
// Recognize:
//   Impossible from B-rep alone.  Returns metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace fiber_draw {

constexpr const char* kSkillId = "fiber_draw";

struct Input
{
    double       final_dia_um      = 125.0;     // standard SMF outer Ø
    double       draw_temp_c       = 2050.0;    // graphite furnace setpoint
    double       draw_speed_m_min  = 1200.0;    // pull rate
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fiber_draw
}  // namespace koocadcam::skill
