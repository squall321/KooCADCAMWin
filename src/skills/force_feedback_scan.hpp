#pragma once
// @lat: [[engine/skills#force_feedback_scan]]
//
// force_feedback_scan — INSPECTION skill: haptic / force-feedback
// contact-scan along the part surface.  A force-controlled stylus
// maintains a constant normal load (~ 50–500 mN typical) while a
// servo carriage sweeps the surface; position is read out at constant
// force.  Differentiated from cmm_scan (discrete touch waypoints) and
// contact_probe_array_scan (parallel probes) by the CONTINUOUS sweep
// under closed-loop force control — ideal for soft materials and
// compliant features where overshoot would damage the surface.
//
//      compliance loop      ┌────────────┐
//   F_set ──▶  servo ──▶  │ haptic stylus│ ── continuous trace ─▶ z(x,y)
//                          └──── F_meas ◀┘
//                                  ▲
//                                  │  closes loop on contact force
//
// What we record:
//   - max_force_n         — actuator force ceiling in newtons (safety cap)
//   - scan_resolution_um  — linear step between sample points
//
// Geometrically a NO-OP.
//
// Input:
//   max_force_n         — > 0; sane range 0.05..50 N
//   scan_resolution_um  — > 0; sane range 0.1..500 µm
//
// DFM checks:
//   max_force_n > 0 (error if not)
//   max_force_n > 10.0 ⇒ warning (high force → surface damage risk)
//   scan_resolution_um > 0 (error if not)
//   scan_resolution_um > 250 ⇒ warning (coarse — close to industrial limit)
//   max_force_n < 0.01 ⇒ info (very low force — limited to optical-grade)
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace force_feedback_scan {

constexpr const char* kSkillId = "force_feedback_scan";

struct Input
{
    double  max_force_n         = 0.5;        // 500 mN typical
    double  scan_resolution_um  = 10.0;       // 10 µm step
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace force_feedback_scan
}  // namespace koocadcam::skill
