#pragma once
// @lat: [[engine/skills#photogrammetry_scan]]
//
// photogrammetry_scan — INSPECTION skill: multi-camera photogrammetric
// reconstruction of the workpiece surface.  N calibrated cameras (or a
// single camera traversing N viewpoints) capture overlapping images;
// SfM + dense MVS yields a triangulated mesh + sparse point cloud, from
// which deviations from the nominal CAD model are computed.
//
//                cam_1   cam_2   cam_3   …   cam_N
//                  ▼       ▼       ▼           ▼
//                   ╲       ╲   │   ╱       ╱
//                    ╲       ╲  │  ╱       ╱
//                       ┌──────┐
//                       │ part │  ← textured targets (or natural texture)
//                       └──────┘
//
// What we record:
//   - camera_count (≥ 2 — minimum for triangulation; ≥ 8 for production)
//   - accuracy_class — "industrial" (~ 0.1 mm), "precision" (~ 0.025 mm),
//                      "metrology" (~ 5 µm; e.g. GOM ATOS / Hexagon)
//   - target_accuracy_mm derived from accuracy_class
//
// Geometrically a NO-OP — like all inspection skills.
//
// Input:
//   camera_count     — number of cameras / viewpoints
//   accuracy_class   — "industrial" | "precision" | "metrology"
//   coverage_pct     — % of part surface visible to ≥ 3 cameras (0..100)
//
// DFM checks:
//   camera_count >= 2 (error if not)
//   camera_count <  4 ⇒ info (low overlap, poor RMS)
//   accuracy_class in known set; unknown ⇒ info
//   coverage_pct in [0, 100]
//   coverage_pct < 60 ⇒ warning (shadows / occlusions will degrade mesh)
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

namespace photogrammetry_scan {

constexpr const char* kSkillId = "photogrammetry_scan";

struct Input
{
    int          camera_count   = 8;
    std::string  accuracy_class = "precision";   // industrial|precision|metrology
    double       coverage_pct   = 85.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace photogrammetry_scan
}  // namespace koocadcam::skill
