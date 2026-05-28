#pragma once
// @lat: [[engine/skills#conveyor_transport]]
//
// conveyor_transport — belt-driven transfer of the workpiece between two
// named stations along the production line.  Geometric no-op: only the
// material-handling step is recorded in the signature so the cell-level
// schedule and cycle-time roll-up can account for transport latency.
//
//      ╭── start_station ──╮         belt @ speed_m_min      ╭── end_station ──╮
//      │                    │ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━▶ │                  │
//      │      part ●        │                                │       ● part     │
//      ╰────────────────────╯       length_m metres          ╰──────────────────╯
//
// Signature pattern:
//   - pattern.kind             = "conveyor_transport"
//   - pattern.start_station    = <string>
//   - pattern.end_station      = <string>
//   - pattern.speed_m_min      = <double>
//   - pattern.length_m         = <double>
//   - pattern.transit_time_s   = (length_m * 60) / speed_m_min
//   - pattern.geometry_changed = false
//
// DFM checks:
//   start_station != empty                       (DFM-INPUT error)
//   end_station   != empty                       (DFM-INPUT error)
//   start_station != end_station                 (DFM-CONVEYOR-DEGENERATE warning)
//   speed_m_min > 0                              (DFM-INPUT error)
//   length_m    > 0                              (DFM-INPUT error)
//   speed_m_min ≤ 60                             (DFM-CONVEYOR-SPEED info — typ industrial)
//   length_m    ≤ 200                            (DFM-CONVEYOR-LENGTH info — cell-scale guard)
//
// Synthesis:
//   NO geometric change.  Compute transit time, clone + stamp.
//
// Recognize:
//   Metadata-only — no geometric fingerprint.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace conveyor_transport {

constexpr const char* kSkillId = "conveyor_transport";

struct Input
{
    std::string  start_station = "ST01";
    std::string  end_station   = "ST02";
    double       speed_m_min   = 10.0;
    double       length_m      = 2.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace conveyor_transport
}  // namespace koocadcam::skill
