#pragma once
// @lat: [[engine/skills#press_fit]]
//
// press_fit — a precisely-sized hole that accepts an interference-fit pin
// or insert.  Geometrically the hole is slightly UNDERSIZED so the pin must
// be pressed in (zero clearance + light interference on the diameter).
//
//                         ┌─ entry_face (planar)
//                         ▼
//                   ──────┬──────────┬──────
//                         │   ▼ axis │
//                         │  ╱│╲     │       ↑ depth_mm
//                         │ ╱ │ ╲    │       │
//                         │   │      │       │   hole_dia =
//                         │   │      │       │     nominal_pin_dia
//                         │   │      │       │     + interference_um × 0.001
//                         └───┴──────┘       ↓     (interference < 0 → hole
//                              ↑                    smaller than pin)
//                         hole_dia (slightly smaller than nominal_pin_dia)
//
// The hole is cylindrical and topologically identical to a drill_hole, but
// with a tightened tolerance class (typ. H7) and an industry-standard
// interference on the diameter (negative interference_um = tight fit).
//
// Conversion convention (industry standard for press-fit specs):
//   hole_dia_mm = nominal_pin_dia_mm + (interference_um × 0.001)
// where interference_um is SIGNED and given in micrometres on the diameter.
// A typical −25 µm interference means the hole is 0.025 mm smaller than the
// pin (= medium press fit, ISO H7/r6 grade).
//
// Signature pattern:
//   - 1 cylindrical face (the bore wall)
//   - 2 circular edges (entry + exit/bottom)
//   - For blind: 1 planar bottom face
//   - pattern.kind == "press_fit", carries nominal_pin_dia / interference / hole_dia
//
// DFM checks:
//   DFM-PF-PIN   : nominal_pin_dia_mm ∈ [1.5, 50] mm
//   DFM-PF-INT   : interference_um ∈ [-100, -5] µm (negative = tight)
//   DFM-INPUT    : depth > 0 for blind holes
//
// Recognize:
//   Cylindrical bore whose diameter matches a standard pin size minus a
//   tight interference (5–100 µm under-size).  Metadata-driven distinction
//   from a plain drill_hole — history-replay first; topology-only matches
//   only when the under-size offset exactly maps to a standard interference
//   class.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace press_fit {

constexpr const char* kSkillId = "press_fit";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm        = 0.0;
    double      position_y_mm        = 0.0;
    gp_Dir      axis_dir             { 0.0, 0.0, -1.0 };
    double      nominal_pin_dia_mm   = 0.0;
    double      interference_um      = -25.0;   // signed; negative = hole smaller than pin
    double      depth_mm             = 0.0;
    bool        through_hole         = false;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Helper exposed for tests: industry-standard press-fit pin sizes (mm).
// Used by recognize() to gate topology-only matches.
const std::vector<double>& standardPinDiameters();

// hole_dia_mm = nominal_pin + interference_um × 0.001
double holeDiameterFor(double nominal_pin_dia_mm, double interference_um);

}  // namespace press_fit
}  // namespace koocadcam::skill
