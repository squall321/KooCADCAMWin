#pragma once
// @lat: [[engine/skills#fertilize_apply]]
//
// fertilize_apply — agricultural fertilizer application record.  Captures
// fertilizer type, NPK ratio (N-P-K, e.g. "20-10-10"), and dose in kg/ha.
// No geometric change.
//
//          ╔════════════════════════════════════════════════════════╗
//          ║                                                        ║
//          ║   ─── broadcaster ╲ ╱ ╲ ╱ ╲ ╱   ← granular spread       ║
//          ║                    │ │ │ │ │                           ║
//          ║                    ▼ ▼ ▼ ▼ ▼                           ║
//          ║   ──── SOIL ──────────────────────────────────         ║
//          ║                                                        ║
//          ║   N : P : K = nutrient ratio                           ║
//          ║   dose_kg_ha × area = total nutrient delivered         ║
//          ║                                                        ║
//          ╚════════════════════════════════════════════════════════╝
//
// Signature pattern:
//   - pattern.kind             = "fertilize_apply"
//   - pattern.fertilizer_type  = string ("urea", "compound", "compost", ...)
//   - pattern.npk_ratio        = string ("20-10-10")
//   - pattern.dose_kg_ha
//   - pattern.geometry_changed = false
//
// DFM checks:
//   fertilizer_type   non-empty                  (DFM-INPUT error)
//   npk_ratio         non-empty + parseable "N-P-K"
//                                                (DFM-FERT-NPK info if not parseable)
//   N+P+K             ≤ 100                      (DFM-FERT-NPK warning if sum > 100 —
//                                                 inconsistent ratio)
//   dose_kg_ha        > 0                        (DFM-INPUT error)
//   dose_kg_ha        ≤ 2000                     (DFM-FERT-DOSE error if higher —
//                                                 exceeds typical agronomic cap)
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

namespace fertilize_apply {

constexpr const char* kSkillId = "fertilize_apply";

struct Input
{
    std::string  fertilizer_type = "compound";
    std::string  npk_ratio       = "20-10-10";
    double       dose_kg_ha      = 200.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fertilize_apply
}  // namespace koocadcam::skill
