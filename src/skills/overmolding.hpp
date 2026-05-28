#pragma once
// @lat: [[engine/skills#overmolding]]
//
// overmolding — apply a SOFT (TPE / TPU / silicone) outer layer onto a RIGID
// substrate.  Distinct from insert_molding by intent:
//   - insert_molding   : non-resin insert (metal, etc.) → resin shell
//   - overmolding      : rigid resin substrate → soft elastomer shell
//
// Both share the same "thin shell" geometric idiom; the differentiator is
// durometer of the outer layer and the soft-touch grip intent.
//
//      ┌────────────────────────────┐   ← TPE / TPU shell (soft, durometer A)
//      │▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒│
//      │▒▒▒▒  ┌──────────────┐  ▒▒▒▒│
//      │▒▒▒▒  │  substrate   │  ▒▒▒▒│   ← rigid ABS / PC / PA
//      │▒▒▒▒  │   (rigid)    │  ▒▒▒▒│
//      │▒▒▒▒  └──────────────┘  ▒▒▒▒│
//      │▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒│
//      └────────────────────────────┘
//             ◄ shell_thickness_mm ►
//
// Geometric model: bbox-MINUS-substrate, then fuse onto the substrate
// (same pattern as insert_molding; common helper).
//
// Signature pattern:
//   pattern.kind               == "overmolding"
//   pattern.substrate_material
//   pattern.overmold_material
//   pattern.durometer_shore_a  (typ. 40–80 for TPE)
//   pattern.shell_thickness_mm
//   pattern.additive           == true
//
// DFM:
//   DFM-OVMOLD-SHELL    : shell_thickness_mm ∈ [0.5, 6.0]
//   DFM-OVMOLD-DUROM    : durometer_shore_a ∈ [20, 95]
//   DFM-OVMOLD-MAT      : substrate_material / overmold_material empty
//
// Recognize: history replay (same caveat as insert_molding).

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace overmolding {

constexpr const char* kSkillId = "overmolding";

struct Input
{
    std::string substrate_material    = "abs";
    std::string overmold_material     = "tpe";
    double      durometer_shore_a     = 60.0;
    double      shell_thickness_mm    = 1.5;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace overmolding
}  // namespace koocadcam::skill
