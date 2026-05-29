#pragma once
// @lat: [[engine/skills#compressor_install]]
//
// compressor_install — surface facility installation of a gas-handling
// compressor (centrifugal or reciprocating), including driver power and
// suction-side design pressure.  This is a facility-installation record;
// the workpiece model (compressor frame or skid) is NOT geometrically
// modified.
//
//        ─────▶  ┌────────────────┐  ─────▶
//                │   compressor   │
//        suction │ (centri | recip)│ discharge
//                │   power_kw     │
//                └────────────────┘
//
// Signature pattern:
//   - pattern.kind                  = "compressor_install"
//   - pattern.compressor_type       = "centrifugal" | "reciprocating"
//   - pattern.power_kw
//   - pattern.suction_pressure_kpa
//   - pattern.geometry_changed      = false
//
// DFM checks:
//   compressor_type ∈ {centrifugal,reciprocating}  (DFM-CI-TYPE info if unknown)
//   power_kw > 0                                   (DFM-INPUT error)
//   power_kw > 50000                               (DFM-CI-POWER info — very large)
//   power_kw < 10                                  (DFM-CI-POWER info — micro class)
//   suction_pressure_kpa > 0                       (DFM-INPUT error)
//   suction_pressure_kpa > 10000                   (DFM-CI-PRESS info — HP suction)
//
// Synthesis:
//   NO geometric change.  Clone workpiece + stamp signature.
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

namespace compressor_install {

constexpr const char* kSkillId = "compressor_install";

struct Input
{
    std::string  compressor_type       = "centrifugal";   // "centrifugal" | "reciprocating"
    double       power_kw              = 2000.0;
    double       suction_pressure_kpa  = 3000.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace compressor_install
}  // namespace koocadcam::skill
