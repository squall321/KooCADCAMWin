#pragma once
// @lat: [[engine/skills#die_attach]]
//
// die_attach — bond a singulated semiconductor die to a substrate / lead frame
// using a thin layer of adhesive (epoxy, AgPaste, or solder).  The macroscopic
// B-rep of the substrate is unchanged; this skill stamps the bonded die +
// adhesive metadata so downstream wire-bond / encap planners can pick up the
// die footprint and the BLT (bond-line thickness).
//
//          die  ────►   ┌─────────────────┐    ← die_size_mm
//                       │                 │
//                       │     silicon     │
//                       └────┬───────┬────┘
//   adhesive  ────►   ▒▒▒▒▒▒│▒▒▒▒▒▒▒│▒▒▒▒▒▒    ← adhesive_type (epoxy / AgPaste / solder)
//                       ────┴───────┴────
//                          lead frame / substrate
//
// Signature pattern:
//   - pattern.kind            = "die_attach"
//   - pattern.adhesive_type   = "epoxy" | "AgPaste" | "solder"
//   - pattern.die_size_mm     = die edge length (square die)
//   - pattern.bond_force_n    = pick-and-place bond force
//   - pattern.geometry_changed = false
//
// DFM checks:
//   die_size_mm     ∈ [0.3, 25.0]            (DFM-DIE-SIZE error if outside)
//   bond_force_n    ∈ [0.05, 30.0]           (DFM-BOND-FORCE error if outside)
//   adhesive_type   ∈ {epoxy,AgPaste,solder} (DFM-ADHESIVE info if unknown)
//
// Synthesis: NO geometric change — clone shape + material, stamp signature.
// Recognize: metadata-only (die + BLT are sub-OCCT material layers).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace die_attach {

constexpr const char* kSkillId = "die_attach";

struct Input
{
    std::string adhesive_type = "epoxy";   // "epoxy" | "AgPaste" | "solder"
    double      die_size_mm   = 5.0;       // square die edge length
    double      bond_force_n  = 1.0;       // pick-and-place compression force
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace die_attach
}  // namespace koocadcam::skill
