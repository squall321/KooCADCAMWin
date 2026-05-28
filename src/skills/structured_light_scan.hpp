#pragma once
// @lat: [[engine/skills#structured_light_scan]]
//
// structured_light_scan — INSPECTION skill: structured-light 3D scan.
// A digital projector projects a sequence of encoded patterns (binary
// Gray code, sinusoidal phase shift, hybrid) onto the part; one or more
// calibrated cameras observe the deformation of those patterns and
// triangulate a dense 3-D point cloud.  Used for cooperative-surface
// dimensional inspection at 5–50 µm accuracy and ~1 Mpt/s throughput.
//
//                 ┌──────────┐
//                 │ projector│  ── encoded pattern stream ─▶
//                 └────┬─────┘
//                      │                  ┌──────┐
//                      ▼                  │ part │  → camera(s)
//                   pattern ────deformed──▶│      │  → triangulated cloud
//                                          └──────┘
//
// What we record:
//   - projector_pattern — encoding scheme (gray|phase_shift|hybrid)
//   - camera_count      — number of synced cameras (1 mono, ≥2 stereo)
//   - accuracy_um       — quoted 1σ point accuracy in µm
//
// Pattern selection guidance (set in DFM):
//   - "gray"        — robust to ambient light, slower (N frames)
//   - "phase_shift" — sub-pixel dense, sensitive to motion (4–N frames)
//   - "hybrid"      — Gray + phase, best accuracy, longest cycle
//
// Geometrically a NO-OP.
//
// Input:
//   projector_pattern — "gray" | "phase_shift" | "hybrid"
//   camera_count      — 1..N (≥ 2 enables stereo cross-check)
//   accuracy_um       — quoted 1σ accuracy in micrometres
//
// DFM checks:
//   camera_count >= 1 (error if not); == 1 ⇒ info (no stereo cross-check)
//   accuracy_um > 0 (error if not)
//   projector_pattern in known set; else ⇒ info
//   accuracy_um > 100 ⇒ warning (typical SL ≤ 50 µm; > 100 µm is loose)
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

namespace structured_light_scan {

constexpr const char* kSkillId = "structured_light_scan";

struct Input
{
    std::string projector_pattern = "phase_shift";  // gray|phase_shift|hybrid
    int         camera_count      = 2;
    double      accuracy_um       = 25.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace structured_light_scan
}  // namespace koocadcam::skill
