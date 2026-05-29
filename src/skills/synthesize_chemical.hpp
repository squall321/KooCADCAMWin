#pragma once
// @lat: [[engine/skills#synthesize_chemical]]
//
// synthesize_chemical — record a wet-lab chemical synthesis reaction.  The
// workpiece (a reactor vessel / flask body) is geometrically unchanged; the
// signature records what reaction was carried out, in what solvent, at what
// temperature, for how long.  Downstream batch records (cf. GMP) join on
// reaction_id.
//
//        ┌─────────────┐                       ┌─────────────┐
//        │  reactants  │   synthesize_chemical │  product    │
//        │  in vessel  │ ─────────────────────▶│  in vessel  │
//        │             │  (rxn_id, T, t, ...)  │             │
//        └─────────────┘                       └─────────────┘
//          shape identical ; reaction metadata stamped
//
// Signature pattern:
//   pattern.kind              = "synthesize_chemical"
//   pattern.reaction_id       = string
//   pattern.reactants_count   = int
//   pattern.solvent           = string
//   pattern.temp_c            = double
//   pattern.time_h            = double
//   pattern.geometry_changed  = false
//
// DFM:
//   DFM-INPUT          reaction_id empty (error)
//   DFM-INPUT          reactants_count < 1 (error)
//   DFM-RXN-TEMP       temp_c outside [-80, 400] C (info)
//   DFM-RXN-TIME       time_h must be > 0 (error)
//   DFM-RXN-SOLVENT    solvent empty (warning)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace synthesize_chemical {

constexpr const char* kSkillId = "synthesize_chemical";

struct Input
{
    std::string  reaction_id     = "RXN-0001";
    int          reactants_count = 2;
    std::string  solvent         = "water";
    double       temp_c          = 25.0;
    double       time_h          = 1.0;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace synthesize_chemical
}  // namespace koocadcam::skill
