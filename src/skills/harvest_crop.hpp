#pragma once
// @lat: [[engine/skills#harvest_crop]]
//
// harvest_crop — agricultural harvest operation record.  Captures the
// crop type, harvest method, expected yield per hectare, and total area
// harvested for downstream production-planning, traceability, and yield
// reporting.  No geometric change is made to the workpiece (which in the
// agriculture context represents a field/plot bookkeeping handle, not a
// machined part).
//
//          ╔════════════════════════════════════════════════════════╗
//          ║                                                        ║
//          ║   ═════════════════════════════════════════════════    ║
//          ║   ║                  CROP FIELD                    ║   ║
//          ║   ║   🌾🌾🌾🌾🌾  area_ha  🌾🌾🌾🌾🌾              ║   ║
//          ║   ║   yield_kg_ha (× area_ha) → total yield        ║   ║
//          ║   ║   harvest_method ∈ {combine, manual, mechanical}║  ║
//          ║   ═════════════════════════════════════════════════    ║
//          ║                                                        ║
//          ╚════════════════════════════════════════════════════════╝
//
// Signature pattern:
//   - pattern.kind             = "harvest_crop"
//   - pattern.crop_type        = string ("wheat", "rice", "corn", ...)
//   - pattern.harvest_method   = string ("combine" | "manual" | "mechanical")
//   - pattern.yield_kg_ha
//   - pattern.area_ha
//   - pattern.total_yield_kg   = yield_kg_ha × area_ha (derived)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   crop_type            non-empty                   (DFM-INPUT error)
//   yield_kg_ha          > 0                         (DFM-INPUT error)
//   area_ha              > 0                         (DFM-INPUT error)
//   harvest_method       ∈ {combine, manual, mechanical}
//                                                    (DFM-HARVEST-METHOD info if unknown)
//   yield_kg_ha          ≤ 15000                     (DFM-HARVEST-YIELD info if higher —
//                                                     extreme yield, verify assumption)
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

namespace harvest_crop {

constexpr const char* kSkillId = "harvest_crop";

struct Input
{
    std::string  crop_type      = "wheat";
    std::string  harvest_method = "combine";      // "combine" | "manual" | "mechanical"
    double       yield_kg_ha    = 3000.0;
    double       area_ha        = 10.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace harvest_crop
}  // namespace koocadcam::skill
