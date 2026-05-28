#pragma once
// @lat: [[engine/skills#etch_silicon]]
//
// etch_silicon — subtractive semiconductor process.  Through the openings
// of a previously developed resist / hard mask, silicon (or other
// substrate) is removed by one of:
//   - DRIE (Bosch)   — deep reactive ion etch, anisotropic, high AR holes
//   - wet            — KOH / TMAH chemical etch, often anisotropic by crystal
//   - dry            — RIE plasma, isotropic-to-anisotropic depending on bias
//
// Geometric change is depth × open-area, but as a metadata-only record we
// preserve the macroscopic shape and store the recipe so downstream
// planners (process flow simulators, cost models) can reconstruct the
// effect.
//
//        ┌──────────────┐  mask           ┌──────────────┐
//        │  substrate   │  + chemistry    │  substrate   │
//        │  (no holes)  │ ──────────────▶ │  + cavities  │
//        └──────────────┘   etch_silicon  └──────────────┘
//
// Signature pattern:
//   - pattern.kind          = "etch_silicon"
//   - pattern.etch_type     = "DRIE" | "wet" | "dry"
//   - pattern.depth_um      = <input>      (must be > 0 — enforced)
//   - pattern.selectivity   = <input>      (mask:Si ratio, dimensionless)
//   - pattern.mask_id       = <input>
//   - pattern.is_subtractive= true
//
// DFM checks:
//   depth_um    > 0           (ERROR — enforced)
//   selectivity > 0
//   mask_id non-empty
//   etch_type ∈ {DRIE, wet, dry}
//   selectivity < 10 with depth_um > 50 → info (mask will be consumed)
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

namespace etch_silicon {

constexpr const char* kSkillId = "etch_silicon";

struct Input
{
    std::string  etch_type   = "DRIE";    // DRIE | wet | dry
    double       depth_um    = 10.0;
    double       selectivity = 50.0;      // mask:Si ratio
    std::string  mask_id     = "M-ETCH-01";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace etch_silicon
}  // namespace koocadcam::skill
