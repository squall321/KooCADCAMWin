#pragma once
// @lat: [[engine/skills#hull_lay]]
//
// hull_lay — shipyard hull-section laying: a major hull block (section) is
// laid down on the keel block / building dock, its steel plates pre-tacked
// to one another, ready for production welding.  The KooCADCAM workpiece
// only carries a representative B-rep of the section envelope; this skill
// records the section identifier, plate thickness, and the principal
// dimensions (length along ship, beam, plate gauge) as metadata.  The
// macroscopic B-rep is unchanged.
//
//        ─── length_m ────────────▶
//        ╔══════════════════════════╗  ▲
//        ║   hull_section_id        ║  │ beam_m
//        ║   plate_thick_mm         ║  │
//        ╚══════════════════════════╝  ▼
//                 keel block
//
// Signature pattern:
//   - pattern.kind             = "hull_lay"
//   - pattern.hull_section_id  = e.g. "S12P" (section 12, port side)
//   - pattern.plate_thick_mm   = shell-plate gauge
//   - pattern.length_m         = section length along ship axis
//   - pattern.beam_m           = section beam (athwartships)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   hull_section_id   non-empty                 (DFM-INPUT error)
//   plate_thick_mm    ∈ [4, 60]                 (DFM-HULL-PLATE error if outside)
//   length_m          > 0                       (DFM-INPUT error)
//   beam_m            > 0                       (DFM-INPUT error)
//   length_m          > 30  ⇒ mega-block, multi-crane lift required
//                                               (DFM-HULL-LEN info)
//
// Synthesis:
//   NO geometric change — clone shape + material verbatim, append signature.
//
// Recognize:
//   Metadata replay only (section identity cannot be inferred from B-rep).

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hull_lay {

constexpr const char* kSkillId = "hull_lay";

struct Input
{
    std::string  hull_section_id = "S01P";   // shipyard section code
    double       plate_thick_mm  = 12.0;     // shell-plate gauge
    double       length_m        = 12.0;     // section length along ship axis
    double       beam_m          = 8.0;      // section beam (athwartships)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hull_lay
}  // namespace koocadcam::skill
