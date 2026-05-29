#pragma once
// @lat: [[engine/skills#agv_transport]]
//
// agv_transport — Autonomous Guided Vehicle (AGV) / AMR floor transfer
// of a part from a named origin station to a named destination station
// over a measured travel distance at a programmed traverse speed.  This
// is a pure material-handling operation: the workpiece geometry is NOT
// modified; only the fleet-frame route is recorded so a downstream fleet
// manager can issue the corresponding mission.
//
//          ╔════════════════════════════════════════════════════════╗
//          ║                                                        ║
//          ║   ┌────────┐  ─── distance_m @ speed_m_s ─▶  ┌────────┐║
//          ║   │ ORIGIN │  ╭───────╮                      │ DEST   │║
//          ║   │ M-01   │  │  AGV  │ ▶▶▶▶▶▶▶▶▶▶▶▶▶▶▶▶▶▶▶▶▶│ A-04   │║
//          ║   └────────┘  ╰───────╯                      └────────┘║
//          ║                                                        ║
//          ╚════════════════════════════════════════════════════════╝
//
// Signature pattern:
//   - pattern.kind             = "agv_transport"
//   - pattern.origin           = string
//   - pattern.destination      = string
//   - pattern.distance_m
//   - pattern.speed_m_s
//   - pattern.geometry_changed = false
//
// DFM checks:
//   origin, destination non-empty               (DFM-INPUT error)
//   origin != destination                       (DFM-AGV-DEGENERATE warning)
//   distance_m > 0                              (DFM-INPUT error)
//   speed_m_s > 0                               (DFM-INPUT error)
//   speed_m_s ≤ 2                               (DFM-AGV-SPEED info — typ floor AGV cap)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
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

namespace agv_transport {

constexpr const char* kSkillId = "agv_transport";

struct Input
{
    std::string  origin       = "M-01";
    std::string  destination  = "A-01";
    double       distance_m   = 25.0;
    double       speed_m_s    = 1.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace agv_transport
}  // namespace koocadcam::skill
