#pragma once
// @lat: [[engine/skills#mold_gate]]
//
// mold_gate — the injection point where molten plastic enters the cavity.
// Geometrically modelled as a small subtractive feature (the actual gate
// tunnel left in the part after demoulding).  Three variants:
//
//   * pin_gate       — small straight cylindrical channel perpendicular to
//                       the entry face.  0.3 – 1.0 mm dia, length ≥ 2× dia.
//
//   * submarine_gate — same as pin_gate but axis angled 10° – 45° from the
//                       entry face normal (so the gate breaks off below the
//                       part surface for cleaner aesthetics).
//
//   * fan_gate       — a flat wedge-shaped sector that spreads flow across
//                       a wide thin entry.  Modelled as a flat box
//                       (dia_or_width × length × 0.5×dia_or_width).
//
// Signature pattern:
//   pattern.kind          = "mold_gate"
//   pattern.gate_type     = "pin_gate" | "submarine_gate" | "fan_gate"
//   pattern.dia_or_width_mm
//
// DFM:
//   DFM-GATE-SIZE   : dia_or_width ∈ [0.3, 2.0] mm
//   DFM-GATE-LEN    : length ≥ 2 × dia_or_width (for cylindrical gates)

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mold_gate {

constexpr const char* kSkillId = "mold_gate";

struct Input
{
    FaceDatum    entry_face;
    double       position_x_mm   = 0.0;
    double       position_y_mm   = 0.0;
    std::string  gate_type       = "pin_gate";  // pin_gate | submarine_gate | fan_gate
    double       dia_or_width_mm = 0.0;
    double       length_mm       = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mold_gate
}  // namespace koocadcam::skill
