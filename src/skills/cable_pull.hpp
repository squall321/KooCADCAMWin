#pragma once
// @lat: [[engine/skills#cable_pull]]
//
// cable_pull — pulling a length of telecom cable (fibre, coax, Cat-6,
// power-over-ethernet) through conduit / tray / ducts.  Records the cable
// type, the pulled length, and the measured/spec pull force.  The
// workpiece B-rep is not modified — only the install record is stamped.
//
//          ┌──────────────────────────────────────────────────────────┐
//          │  conduit / cable tray                                     │
//          │                                                           │
//          │  ╔═══╗──────────  cable (length_m @ pull_force_n)  ─────▶│
//          │  ║ A ║                                                    │
//          │  ╚═══╝                                                    │
//          │                                                           │
//          └──────────────────────────────────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "cable_pull"
//   - pattern.cable_type       = string ("fibre_smf" | "cat6" | "coax_rg6" | ...)
//   - pattern.length_m
//   - pattern.pull_force_n
//   - pattern.geometry_changed = false
//
// DFM checks:
//   cable_type non-empty            (DFM-INPUT error)
//   length_m   > 0                  (DFM-INPUT error)
//   pull_force_n > 0                (DFM-INPUT error)
//   pull_force_n > 600 N            (DFM-CABLE-FORCE warning — typ telecom limit)
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

namespace cable_pull {

constexpr const char* kSkillId = "cable_pull";

struct Input
{
    std::string  cable_type    = "fibre_smf";
    double       length_m      = 50.0;
    double       pull_force_n  = 100.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cable_pull
}  // namespace koocadcam::skill
