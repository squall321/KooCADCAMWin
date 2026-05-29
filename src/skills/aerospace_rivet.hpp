#pragma once
// @lat: [[engine/skills#aerospace_rivet]]
//
// aerospace_rivet — installation record for a structural fastener that joins
// thin aerospace skins.  Unlike `rivet_hole` (which DRILLS the through-bore),
// this skill captures the INSTALLED rivet metadata: the head sits flush in
// a previously machined countersink, the body fills the through-hole, and
// the tail is upset (solid) or bulbed (blind).  The macroscopic B-rep is
// already correct after rivet_hole + countersink; this skill ONLY stamps
// the installed-fastener record so process planning and inspection can
// downstream verify torque/upset.
//
//            ─────────┬─────────┐
//          ──────────╲│╱──────  │     ← flush head (sunk into countersink)
//                    ╲│╱        │
//                     │         │     ← body (Ø rivet_dia, length length)
//                     │         │
//          ──────────╱│╲──────  │
//            ─────────┴─────────┘
//                     ↑
//                  upset / bulb (solid or blind rivet tail)
//
// Signature pattern:
//   - pattern.kind             = "aerospace_rivet"
//   - pattern.rivet_dia_mm     = body Ø
//   - pattern.length_mm        = grip length (skin-stack thickness + tail)
//   - pattern.alloy            = "AA2117" | "AA2017" | "AA5056" | "monel" | …
//   - pattern.style            = "blind" | "solid"
//   - pattern.head_style       = "flush_100deg"   (aerospace structural skins)
//   - geometry_changed         = false
//
// DFM checks:
//   rivet_dia_mm  ∈ [2.4, 8.0]            (DFM-RIVET-DIA error if outside)
//   length_mm     ≥ 1.5 × rivet_dia_mm    (DFM-RIVET-LEN error if too short)
//   length_mm     ≤ 10  × rivet_dia_mm    (DFM-RIVET-LEN error if grossly long)
//   alloy         non-empty               (DFM-INPUT warning)
//   style         ∈ {"blind", "solid"}    (DFM-RIVET-STYLE info if unknown)
//
// Synthesis:
//   NO geometric change — the through-hole + countersink are already in
//   place from prior skills.  This skill clones the workpiece and stamps
//   the installed-fastener signature.
//
// Recognize:
//   The installed shank is sub-OCCT (a separate body — no Boolean fuse).
//   Recognition is metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace aerospace_rivet {

constexpr const char* kSkillId = "aerospace_rivet";

struct Input
{
    FaceDatum   entry_face;                       // face the head sits flush against
    double      position_x_mm = 0.0;              // installed centre on entry face
    double      position_y_mm = 0.0;
    double      rivet_dia_mm  = 4.0;              // body shank Ø (e.g. 3.2, 4.0, 4.8 mm)
    double      length_mm     = 10.0;             // grip length (skin stack + tail)
    std::string alloy         = "AA2117";         // "AA2117" | "AA2017" | "AA5056" | "monel"
    std::string style         = "solid";          // "blind" | "solid"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace aerospace_rivet
}  // namespace koocadcam::skill
