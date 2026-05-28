#pragma once
// @lat: [[engine/skills#contact_probe_array_scan]]
//
// contact_probe_array_scan — INSPECTION skill: MULTI-PROBE contact
// scanning.  Unlike cmm_scan (caller-supplied waypoints, single probe),
// this skill records a parallel probe array (e.g. 4×4, 8×8 ruby-tip
// fixtures used for in-line dimensional gauging) plus a sweep pattern
// that traverses the part.  Production use: gear teeth, blisk roots,
// stamping checks where N probes drop simultaneously per cycle.
//
//      probe_1  probe_2  …  probe_N
//        │        │            │
//        ▼        ▼            ▼
//       ┌─────────────────────────┐
//       │       part surface      │  (sweep_pattern = raster / spiral / serpentine)
//       └─────────────────────────┘
//
// What we record:
//   - probe_count    — number of contact probes in the array
//   - probe_dia_mm   — ruby ball diameter (uniform across array)
//   - sweep_pattern  — "raster" | "spiral" | "serpentine"
//
// Geometrically a NO-OP.
//
// Input:
//   probe_count    — ≥ 1; arrays typically 4..256
//   probe_dia_mm   — > 0; typical 1..4 mm
//   sweep_pattern  — "raster" | "spiral" | "serpentine"
//
// DFM checks:
//   probe_count >= 1 (error if not)
//   probe_count > 256 ⇒ warning (impractical fixture mass / calibration)
//   probe_dia_mm > 0 (error if not)
//   sweep_pattern in known set; else ⇒ info
//   probe_dia_mm < 0.5 ⇒ warning (fragile ruby tip — high breakage risk)
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

namespace contact_probe_array_scan {

constexpr const char* kSkillId = "contact_probe_array_scan";

struct Input
{
    int         probe_count    = 16;
    double      probe_dia_mm   = 2.0;
    std::string sweep_pattern  = "raster";    // raster | spiral | serpentine
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace contact_probe_array_scan
}  // namespace koocadcam::skill
