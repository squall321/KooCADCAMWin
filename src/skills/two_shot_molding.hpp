#pragma once
// @lat: [[engine/skills#two_shot_molding]]
//
// two_shot_molding — two-material injection molding in a single mold.  A
// first material (e.g. rigid skeleton) is injected, the core retracts to
// expose a second cavity, and a second material (e.g. soft grip) is
// injected against the first.  The final part is BICOMPONENT but the
// geometric shape is treated as a single workpiece — the material
// distribution is encoded in feature metadata only.
//
//      ┌─────── material_1 (rigid) ──────────────────┐
//      │░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│
//      │░░░░░░░░░░  ────────────────────   ░░░░░░░░░░░│   ← parting geometry
//      │░░░░░░░░░░  ▓▓▓▓ material_2 ▓▓▓▓   ░░░░░░░░░░░│     (interface)
//      │░░░░░░░░░░  ────────────────────   ░░░░░░░░░░░│
//      │░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│
//      └──────────────────────────────────────────────┘
//
// METADATA-ONLY (geometric no-op).  Bonding chemistry, parting-line design
// and core retraction are recorded in the signature so a downstream
// process planner / mold designer can reproduce the tooling.
//
// Signature pattern:
//   pattern.kind                == "two_shot_molding"
//   pattern.material_1
//   pattern.material_2
//   pattern.parting_geometry    ∈ {"planar", "stepped", "complex"}
//   pattern.geometric_change    == false
//
// DFM:
//   DFM-2SHOT-MATERIAL : material_1 or material_2 missing
//   DFM-2SHOT-PARTING  : parting_geometry not one of the recognised hints
//                        (info-only — proceeds anyway)
//   DFM-2SHOT-COMPAT   : material_1 / material_2 known to bond poorly
//                        (info-only)
//
// Recognize: metadata replay only.  No B-Rep clue distinguishes a 2-shot
// part from a single-shot one at the analytic-shape level.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace two_shot_molding {

constexpr const char* kSkillId = "two_shot_molding";

struct Input
{
    std::string material_1          = "abs";
    std::string material_2          = "tpe";
    std::string parting_geometry    = "planar";   // "planar" | "stepped" | "complex"
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace two_shot_molding
}  // namespace koocadcam::skill
