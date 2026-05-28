#pragma once
// @lat: [[engine/skills#burnish]]
//
// burnish — cold-work surface refinement.  A polished hardened roller or
// ball is pressed against the target face under controlled load, plastically
// smearing surface asperities and densifying the near-surface layer.  This
// drops Ra by 5 – 10x without removing material and adds compressive
// residual stress similar to shot-peening.
//
//        ┌─────────────────────┐
//        │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │  ← smoothed, work-hardened layer
//        │   (unchanged bulk)  │
//        └─────────────────────┘
//          macroscopic shape ≈ identical;
//          Ra ↓ (e.g., 1.6 → 0.2 μm)
//
// Signature pattern:
//   - pattern.kind            = "burnish"
//   - pattern.target_face_id  = face that was burnished
//   - pattern.pressure_kn     = tool contact load
//   - pattern.target_ra_um    = post-burnish surface roughness target
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT                pressure_kn ≤ 0 / target_ra_um ≤ 0  → error
//   DFM-BURNISH-PRESSURE     pressure_kn ∉ [0.1, 20.0]           → error
//   DFM-BURNISH-RA           target_ra_um ∉ [0.05, 3.2]          → error
//
// Synthesis:
//   NO geometric change.  Clones the workpiece carrying a new signature.
//
// Recognize:
//   Metadata replay only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace burnish {

constexpr const char* kSkillId = "burnish";

struct Input
{
    FaceDatum    entry_face;
    double       pressure_kn   = 2.0;    // tool contact load
    double       target_ra_um  = 0.4;    // resulting Ra
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace burnish
}  // namespace koocadcam::skill
