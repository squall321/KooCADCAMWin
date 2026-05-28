#pragma once
// @lat: [[engine/skills#optical_3d_scan]]
//
// optical_3d_scan — INSPECTION skill: pure camera-only triangulation
// 3-D scan.  Unlike structured_light_scan (active illumination) or
// photogrammetry_scan (SfM/MVS over many viewpoints), this skill
// records a fixed STEREO camera pair (passive triangulation under
// scene lighting) characterised by baseline and per-pixel resolution.
// Used as a low-cost in-line check for parts with natural texture.
//
//           cam_L ─── baseline_mm ─── cam_R
//             ╲                       ╱
//              ╲    epipolar match    ╱
//               ▼                    ▼
//                ┌──────────────────┐
//                │    part surface  │
//                └──────────────────┘
//
// What we record:
//   - baseline_mm    — stereo baseline (eye separation), drives accuracy
//   - resolution_um  — projected pixel footprint on the part (µm/pixel)
//
// Accuracy heuristic (recorded in pattern):
//   accuracy_um ≈ resolution_um / (baseline_mm / working_distance_mm)
//   We do not require working_distance, so we expose only the inputs.
//
// Geometrically a NO-OP.
//
// Input:
//   baseline_mm    — > 0; typical 50..300 mm
//   resolution_um  — > 0; per-pixel footprint on the part
//
// DFM checks:
//   baseline_mm > 0 (error if not)
//   resolution_um > 0 (error if not)
//   baseline_mm < 20 ⇒ warning (too short → poor depth resolution)
//   resolution_um > 200 ⇒ warning (very coarse — likely not metrology-grade)
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace optical_3d_scan {

constexpr const char* kSkillId = "optical_3d_scan";

struct Input
{
    double  baseline_mm    = 120.0;
    double  resolution_um  = 50.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace optical_3d_scan
}  // namespace koocadcam::skill
