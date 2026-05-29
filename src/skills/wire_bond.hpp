#pragma once
// @lat: [[engine/skills#wire_bond]]
//
// wire_bond — form ball/wedge bonds joining die pads to package leads via a
// fine metallic wire (Au, Cu, or Al).  Each bond is a sub-OCCT wire loop, so
// the package B-rep does not change; the skill stamps wire_count and material
// metadata for downstream electrical / encap planning.
//
//        die pad ●═══════════╗      ← ball bond on die
//                            ║
//                         wire loop (wire_dia_um)
//                            ║
//                            ╚═════● lead    ← wedge bond on lead
//
// Signature pattern:
//   - pattern.kind            = "wire_bond"
//   - pattern.wire_dia_um     = wire diameter in micrometres
//   - pattern.wire_material   = "Au" | "Cu" | "Al"
//   - pattern.bond_count      = total bonds (== 2 × wires for ball+wedge)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   wire_dia_um   ∈ [15, 75]          (DFM-WIRE-DIA error if outside)
//   wire_material ∈ {Au, Cu, Al}      (DFM-WIRE-MAT info if unknown)
//   bond_count    > 0                  (DFM-INPUT error if non-positive)
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

namespace wire_bond {

constexpr const char* kSkillId = "wire_bond";

struct Input
{
    double      wire_dia_um   = 25.0;   // typical 18 / 25 / 33 / 50 um
    std::string wire_material = "Au";   // "Au" | "Cu" | "Al"
    int         bond_count    = 16;     // total bonds (both ends counted)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace wire_bond
}  // namespace koocadcam::skill
