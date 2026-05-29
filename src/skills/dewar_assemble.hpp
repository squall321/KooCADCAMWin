#pragma once
// @lat: [[engine/skills#dewar_assemble]]
//
// dewar_assemble — captures the assembly record for a vacuum-jacketed
// cryogenic vessel ("Dewar").  Two concentric shells with the annular
// space pumped to high vacuum and (usually) filled with multilayer
// insulation (MLI) or aerogel.  The B-rep of the inner liner and the
// outer jacket are produced by upstream skills (form/weld/etc.); this
// skill stamps the as-assembled record: inner Ø, jacket wall, and
// insulation type.
//
//        ┌──────── outer jacket ────────┐
//        │ ┌────── vacuum + MLI ──────┐ │
//        │ │ ┌──── inner liner ─────┐ │ │
//        │ │ │                      │ │ │
//        │ │ │   cryogen (LN2/LHe)  │ │ │
//        │ │ │                      │ │ │
//        │ │ └──────────────────────┘ │ │
//        │ └──────────────────────────┘ │
//        └──────────────────────────────┘
//        ↑   ↑                          ↑
//      jacket    vacuum_jacket_thickness_mm
//
// Signature pattern:
//   - pattern.kind                       = "dewar_assemble"
//   - pattern.inner_dia_mm
//   - pattern.vacuum_jacket_thickness_mm
//   - pattern.insulation_type            = "MLI" | "aerogel" | "perlite" | "none"
//   - geometry_changed                   = false
//
// DFM checks:
//   inner_dia_mm                   > 0                     (DFM-INPUT)
//   vacuum_jacket_thickness_mm     ∈ [5, 200]              (DFM-DEWAR-GAP)
//   insulation_type                in known set            (DFM-DEWAR-INS info)
//
// Synthesis: clone + stamp.  Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace dewar_assemble {

constexpr const char* kSkillId = "dewar_assemble";

struct Input
{
    double      inner_dia_mm                 = 200.0;
    double      vacuum_jacket_thickness_mm   = 25.0;
    std::string insulation_type              = "MLI";   // "MLI" | "aerogel" | "perlite" | "none"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace dewar_assemble
}  // namespace koocadcam::skill
