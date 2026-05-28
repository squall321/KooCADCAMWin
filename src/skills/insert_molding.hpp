#pragma once
// @lat: [[engine/skills#insert_molding]]
//
// insert_molding — injection-mold a resin SHELL around a pre-placed metal
// (or other) INSERT part already loaded into the mold cavity.  Common for
// threaded brass inserts, IC-package leadframes, stiff structural skeletons
// over-coated with plastic, etc.
//
//      ┌────────────────────────────┐   ← resin shell (additive)
//      │░░░░░░░░░░░░░░░░░░░░░░░░░░░░│
//      │░░░░  ┌──────────────┐  ░░░░│
//      │░░░░  │   insert     │  ░░░░│   ← workpiece (the insert)
//      │░░░░  │   (e.g. Cu)  │  ░░░░│
//      │░░░░  └──────────────┘  ░░░░│
//      │░░░░░░░░░░░░░░░░░░░░░░░░░░░░│
//      └────────────────────────────┘
//             ◄ shell_thickness_mm ►
//
// Geometric model (slice-1 approximation): the resin shell is approximated
// as (expanded-bbox MINUS input) — i.e. a rectangular slab grown out from
// the workpiece's bbox by shell_thickness_mm on all six sides, with the
// insert subtracted out so the original shape is preserved.  The result is
// then fused onto the original insert (so the workpiece is now insert+resin).
//
// A higher-fidelity slice-2 could use BRepOffsetAPI_MakeOffsetShape; bbox-
// minus-insert is safe and matches the weld_buildup pattern (additive slab
// fused onto parent).
//
// Signature pattern:
//   pattern.kind                 == "insert_molding"
//   pattern.insert_material
//   pattern.resin_material
//   pattern.shell_thickness_mm
//   pattern.additive             == true
//   pattern.encapsulates_insert  == true
//
// DFM:
//   DFM-INSERT-SHELL : shell_thickness_mm ∈ [0.5, 10.0]
//   DFM-INSERT-MAT   : missing insert_material or resin_material
//
// Recognize:
//   Primary: replay from feature history.
//   Fallback: detect a thin uniform "skin" around an inner solid is hard
//   without the insert reference shape — slice-1 only does history replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace insert_molding {

constexpr const char* kSkillId = "insert_molding";

struct Input
{
    std::string insert_material      = "brass_360";
    std::string resin_material       = "abs";
    double      shell_thickness_mm   = 2.0;
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace insert_molding
}  // namespace koocadcam::skill
