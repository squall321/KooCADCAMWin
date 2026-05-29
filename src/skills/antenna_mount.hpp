#pragma once
// @lat: [[engine/skills#antenna_mount]]
//
// antenna_mount — installation record for a telecom antenna mounted on a
// mast / tower / rooftop bracket.  Captures antenna type, the mount
// elevation above ground, and the azimuth bearing (compass direction the
// main lobe points).  The workpiece B-rep is unchanged — only the bracket-
// frame install record is stamped.
//
//                         ╱│   ← antenna (azimuth_deg)
//                        ╱ │
//                        ──┤
//                          │
//                          │   ← mast (mount_height_m)
//                          │
//                          │
//                       ───┴───   ← ground
//
// Signature pattern:
//   - pattern.kind             = "antenna_mount"
//   - pattern.antenna_type     = "panel" | "omni" | "yagi" | "parabolic" | ...
//   - pattern.mount_height_m
//   - pattern.azimuth_deg      ∈ [0, 360)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   antenna_type non-empty           (DFM-INPUT error)
//   mount_height_m > 0               (DFM-INPUT error)
//   mount_height_m > 100             (DFM-ANT-HEIGHT warning — tall tower review)
//   azimuth_deg ∈ [0, 360)           (DFM-ANT-AZIMUTH error if outside)
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

namespace antenna_mount {

constexpr const char* kSkillId = "antenna_mount";

struct Input
{
    std::string  antenna_type    = "panel";
    double       mount_height_m  = 30.0;
    double       azimuth_deg     = 0.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace antenna_mount
}  // namespace koocadcam::skill
