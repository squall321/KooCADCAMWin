#pragma once
// @lat: [[engine/skills#fabric_cutting]]
//
// fabric_cutting — cut pre-finished fabric from a roll along marker patterns
// to produce stacked garment / accessory blanks ready for sewing.  Method is
// independent of the marker (rotary blade, laser, ultrasonic, die-press);
// what matters at the planning layer is which pattern is being cut, the
// linear consumption from the roll, and the expected scrap factor.
//
// The OCCT-side workpiece is the fabric roll; the produced blanks are
// described by the marker pattern_id.  This is metadata-only — the
// boundary representation is unchanged in the planner abstraction.
//
//              ┌────────────────────────────────────┐
//   roll ─────►│ ▰ ▰ ▰ ▰ ▰ ▰ ▰ ▰ ▰ ▰ ▰ ▰ ▰ ▰ ▰ ▰ │  marker pattern_id
//              │      cut_length_m  ═══════>        │
//              └────────────────────────────────────┘
//
// Signature pattern:
//   - pattern.kind             = "fabric_cutting"
//   - pattern.pattern_id       = marker / cut-file identifier
//   - pattern.cut_length_m     = metres consumed from the roll
//   - pattern.waste_pct        = expected scrap percentage (0..100)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   pattern_id non-empty      (DFM-INPUT          error)
//   cut_length_m > 0          (DFM-INPUT          error)
//   waste_pct ∈ [0, 100]      (DFM-INPUT          error)
//   waste_pct ≤ 20            (DFM-TEX-WASTE      info — re-nest marker)
//
// Synthesis:
//   NO geometric change.  Clone + stamp.
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

namespace fabric_cutting {

constexpr const char* kSkillId = "fabric_cutting";

struct Input
{
    std::string  pattern_id    = "MARKER-DEFAULT";
    double       cut_length_m  = 5.0;             // metres consumed from roll
    double       waste_pct     = 12.0;            // expected scrap %
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fabric_cutting
}  // namespace koocadcam::skill
