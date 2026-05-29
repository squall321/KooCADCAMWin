#pragma once
// @lat: [[engine/skills#countersunk_bolt_seat]]
//
// countersunk_bolt_seat — COMPOUND fastener-seat for a flat-head countersunk
// machine screw (ISO 7046 / DIN 965).  Chains three primitive cuts so the
// head of the bolt sits flush with the entry face and the threads engage
// the receiving part below:
//
//   sub-feature 1: through pilot drill (clearance for shank)
//   sub-feature 2: conical countersink (82° UNC / 90° metric / 100° aero)
//   sub-feature 3: (optional) small flat-bottom cylindrical relief at the
//                  cone-to-pilot junction so the screw head bottoms on the
//                  cone walls, not the junction radius.
//
// All three cuts are concentric on the same axis and are fused into a single
// connected cutter before subtracting from the workpiece (counterbore pattern)
// to avoid OCCT misclassifying overlapping concentric solids.
//
//                           ┌─ entry_face (planar)
//                           ▼
//                     ──────┬─────────────────┬──────
//                           │\               /│
//                           │ \   csk cone  / │  ← sub-feature 2
//                           │  \           /  │
//                           │   \         /   │
//                           │    └───┬───┘    │  ← sub-feature 3 (optional flat)
//                           │        │        │
//                           │   pilot dia     │  ← sub-feature 1 (through)
//                           │        │        │
//                           │        │        │
//                           └────────┴────────┘
//
// Input lookup tables (M-spec):
//
//   M-spec  | clearance  | head dia  | recommended csk angle
//           | (pilot)    | (top dia) |
//   --------+------------+-----------+-----------------------
//   M2      | 2.4        | 3.8       | 90°
//   M2.5    | 2.9        | 4.7       | 90°
//   M3      | 3.4        | 5.6       | 90°
//   M4      | 4.5        | 7.5       | 90°
//   M5      | 5.5        | 9.2       | 90°
//   M6      | 6.6        | 11.0      | 90°
//   M8      | 9.0        | 14.5      | 90°
//
// 82° is selected when csk_angle = 82 (UNC).  100° is allowed for aerospace.
//
// DFM checks:
//   DFM-002              : derived pilot < 0.8 mm
//   DFM-CSK-SIZE         : fastener_size unknown
//   DFM-CSK-ANGLE        : csk_angle outside {82, 90, 100}
//   DFM-CSK-THICKNESS    : stock thickness < head_dia × 0.6 (head sinks too deep)
//
// Recognize: metadata replay (compound fastener seats are hard to fingerprint
//            from B-rep alone without ambiguity vs. a plain countersink).
//            confidence = 1.0 for history-matched candidates.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace countersunk_bolt_seat {

constexpr const char* kSkillId = "countersunk_bolt_seat";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    std::string fastener_size;            // "M2".."M8"
    double      csk_angle_deg  = 90.0;    // 82 / 90 / 100
    bool        with_relief    = false;   // optional flat-bottom relief
};

// Lookup helpers — return 0 if size unknown.
double clearanceDiameterFor (const std::string& size);
double headDiameterFor      (const std::string& size);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace countersunk_bolt_seat
}  // namespace koocadcam::skill
