#pragma once
// @lat: [[engine/skills#drivetrain_install]]
//
// drivetrain_install — engine + transmission + driveline marriage to the
// chassis on the automotive trim line.  Records drivetrain configuration
// (FWD / RWD / AWD), engine + transmission identifiers, and stamps the
// installation metadata.  Macroscopic B-rep is unchanged at this stage —
// the powertrain is a sub-assembly with its own history.
//
//     ┌─────────┐   ┌─────────┐                  ┌────────────────┐
//     │ engine  │ + │  trans  │  +  driveline ─▶ │ chassis (FWD/  │
//     │  E-xxx  │   │  T-xxx  │   half-shafts    │  RWD/AWD)      │
//     └─────────┘   └─────────┘                  └────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "drivetrain_install"
//   - pattern.drivetrain_type  = "FWD" | "RWD" | "AWD"
//   - pattern.engine_id        = string
//   - pattern.transmission_id  = string
//   - pattern.geometry_changed = false
//
// DFM checks:
//   drivetrain_type ∈ {FWD, RWD, AWD}              (DFM-DRIVETRAIN-TYPE error)
//   engine_id non-empty                            (DFM-INPUT error)
//   transmission_id non-empty                      (DFM-INPUT error)
//
// Synthesis:
//   NO geometric change.  Clone shape + material, stamp signature.
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

namespace drivetrain_install {

constexpr const char* kSkillId = "drivetrain_install";

struct Input
{
    std::string  drivetrain_type = "FWD";    // "FWD" | "RWD" | "AWD"
    std::string  engine_id       = "E-2000-G4";
    std::string  transmission_id = "T-CVT-A1";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace drivetrain_install
}  // namespace koocadcam::skill
