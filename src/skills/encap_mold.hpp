#pragma once
// @lat: [[engine/skills#encap_mold]]
//
// encap_mold — transfer-mold an epoxy molding compound (EMC) over the bonded
// die + wires to form the finished IC body.  Even though the package body
// IS physical, in this skill we treat the macroscopic substrate B-rep as
// unchanged: the EMC envelope is registered as metadata so the downstream
// dicing / marking planners can find the package boundary by signature
// rather than by Boolean fuse (which would explode the OCCT model).
//
//          ┌───────────────────────┐   ← EMC envelope (molding_compound)
//          │   ╔═════════════╗    │
//          │   ║   die+wires ║    │
//          │   ╚═════════════╝    │
//          └─┬───────────────────┬─┘
//        substrate (unchanged B-rep)
//
// Signature pattern:
//   - pattern.kind                   = "encap_mold"
//   - pattern.molding_compound       = e.g. "EMC_G700" | "EMC_KMC184"
//   - pattern.cavity_count           = mold cavities per shot
//   - pattern.transfer_pressure_kpa  = ram pressure on EMC
//   - pattern.geometry_changed       = false
//
// DFM checks:
//   cavity_count          ∈ [1, 256]              (DFM-CAVITY error)
//   transfer_pressure_kpa ∈ [3000, 12000]         (DFM-PRESSURE error)
//   molding_compound      non-empty               (DFM-COMPOUND info)
//
// Synthesis: NO geometric change — clone shape + material, stamp signature.
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace encap_mold {

constexpr const char* kSkillId = "encap_mold";

struct Input
{
    std::string molding_compound      = "EMC_G700";
    int         cavity_count          = 64;
    double      transfer_pressure_kpa = 7000.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace encap_mold
}  // namespace koocadcam::skill
