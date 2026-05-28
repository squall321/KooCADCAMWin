#pragma once
// @lat: [[engine/skills#lidar_scan]]
//
// lidar_scan — INSPECTION skill: LIDAR (light detection and ranging)
// point-cloud scan.  A pulsed laser sweeps the volume around the part
// (single-axis, multi-axis, or rotating mirror) and time-of-flight
// (or AMCW phase) yields a sparse-to-medium-density 3-D point cloud.
// Suited for large-volume capture (≥ 1 m parts, fixturing, reverse-
// engineering of installed assemblies) where fringe / structured-light
// scanners cannot reach the working distance.
//
//          ┌─────┐
//          │laser│ ─── pulsed beam ───▶ │ part │ → return pulse
//          └─────┘                       └──────┘   τ → range
//             ▲
//             │ rotating prism / galvo → angular sweep
//
// What we record:
//   - range_m         — maximum unambiguous range in metres
//   - points_per_sec  — pulse repetition × scan rate
//   - ranging_class   — "ToF" (time-of-flight) | "AMCW" (phase) | "FMCW"
//
// Geometrically a NO-OP.
//
// Input:
//   range_m         — 0.1..1000 m typical
//   points_per_sec  — > 0; production scanners 1e5..2e6 pt/s
//   ranging_class   — "ToF" | "AMCW" | "FMCW"
//
// DFM checks:
//   range_m > 0 and ≤ 1000 (error outside)
//   points_per_sec > 0 (error if not)
//   ranging_class in known set; unknown ⇒ info
//   points_per_sec < 10 000 ⇒ warning (too sparse for industrial QA)
//   range_m < 0.3 ⇒ info (LIDAR overkill — use SL / interferometry)
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace lidar_scan {

constexpr const char* kSkillId = "lidar_scan";

struct Input
{
    double      range_m         = 30.0;        // 30 m default coverage
    double      points_per_sec  = 5.0e5;       // 500 k pt/s mid-class
    std::string ranging_class   = "ToF";       // ToF | AMCW | FMCW
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace lidar_scan
}  // namespace koocadcam::skill
