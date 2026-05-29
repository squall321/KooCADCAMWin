#pragma once
// @lat: [[engine/skills#planting]]
//
// planting — agricultural seeding operation record.  Captures seed type,
// seed density (seeds per m²) and planting depth.  No geometric change
// to the workpiece (the field handle).  Used downstream for germination
// forecasting, input-cost accounting, and crop-rotation traceability.
//
//          ╔════════════════════════════════════════════════════════╗
//          ║                                                        ║
//          ║   ──── SOIL SURFACE ──────────────────────────────     ║
//          ║         │      │      │      │      │                  ║
//          ║         ▼      ▼      ▼      ▼      ▼                  ║
//          ║       ┌─┴─┐  ┌─┴─┐  ┌─┴─┐  ┌─┴─┐  ┌─┴─┐                ║
//          ║       │   │  │   │  │   │  │   │  │   │ ← planting_depth_mm
//          ║       └───┘  └───┘  └───┘  └───┘  └───┘                ║
//          ║                                                        ║
//          ║   seed_density_per_m2 controls inter-seed spacing      ║
//          ║                                                        ║
//          ╚════════════════════════════════════════════════════════╝
//
// Signature pattern:
//   - pattern.kind                 = "planting"
//   - pattern.seed_type            = string
//   - pattern.seed_density_per_m2
//   - pattern.planting_depth_mm
//   - pattern.geometry_changed     = false
//
// DFM checks:
//   seed_type                  non-empty                  (DFM-INPUT error)
//   seed_density_per_m2        > 0                        (DFM-INPUT error)
//   planting_depth_mm          > 0                        (DFM-INPUT error)
//   planting_depth_mm          ≤ 200                      (DFM-PLANT-DEPTH error if deeper —
//                                                          seed can't reach surface)
//   seed_density_per_m2        ≤ 2000                     (DFM-PLANT-DENSITY info if higher —
//                                                          overcrowding, verify)
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

namespace planting {

constexpr const char* kSkillId = "planting";

struct Input
{
    std::string  seed_type            = "wheat";
    double       seed_density_per_m2  = 400.0;
    double       planting_depth_mm    = 30.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace planting
}  // namespace koocadcam::skill
