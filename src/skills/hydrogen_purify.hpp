#pragma once
// @lat: [[engine/skills#hydrogen_purify]]
//
// hydrogen_purify — purification of an H2 product stream.  Three common
// industrial methods are supported:
//
//   PSA          Pressure-Swing Adsorption (activated carbon / zeolite beds)
//                Typical output: 99.99 % @ 100..50000 Nm3/h
//
//   cryogenic    Liquefaction & flash distillation, removes CH4/CO/N2
//                Typical output: 99.9999 % @ large scale
//
//   electrolysis Electrochemical purification (palladium membrane / PEM)
//                Typical output: 99.99999 % @ small scale
//
// At the CAD level this skill is a unit-operation tag — the workpiece
// (e.g., a pressure vessel CAD) is unchanged, but the H2 mass flow
// quality + throughput are recorded for downstream BoP sizing.
//
// Signature pattern:
//   - pattern.kind             = "hydrogen_purify"
//   - pattern.method           = "PSA" | "cryogenic" | "electrolysis"
//   - pattern.purity_pct
//   - pattern.flow_nm3_h
//   - pattern.geometry_changed = false
//
// DFM checks:
//   method in known set                      (DFM-H2-METHOD info if unknown)
//   purity_pct > 0 AND < 100                 (DFM-INPUT error)
//   purity_pct ≥ 99.0                        (DFM-H2-PURITY info — low for industrial H2)
//   flow_nm3_h > 0                           (DFM-INPUT error)
//   method-specific throughput envelope      (DFM-H2-FLOW info)
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

namespace hydrogen_purify {

constexpr const char* kSkillId = "hydrogen_purify";

struct Input
{
    std::string  method     = "PSA";   // "PSA" | "cryogenic" | "electrolysis"
    double       purity_pct = 99.99;
    double       flow_nm3_h = 1000.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hydrogen_purify
}  // namespace koocadcam::skill
