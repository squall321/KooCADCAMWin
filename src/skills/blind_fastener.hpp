#pragma once
// @lat: [[engine/skills#blind_fastener]]
//
// blind_fastener — a "pop-rivet" style fastener installed from ONE side of
// the assembly only.  Differs from rivet_hole in that:
//   - the diameter table is the standard BLIND RIVET set ({3.2, 4.0, 4.8,
//     6.4} mm, not generic rivet sizes)
//   - the install spec includes a grip length range
//   - tooling is recorded as a rivet gun (blind-side mandrel pull), not a
//     bucking-bar set
//
//                            ▼ install direction (axis)
//                      ┌─────┐                       ▲
//                      │   ⨯ │  ← rivet body          │ joint stack
//                      │     │                        │  (= grip_length_mm
//          ────────────┴─────┴──────────              │   ≈ workpiece thickness)
//                                                     ▼
//                         ↑
//                  drilled through-hole
//                  (dia ≈ nominal_dia + 0.05 mm)
//
// Slice-1 geometry: a clean cylindrical through-hole sized slightly above
// the nominal rivet body so the rivet can be inserted.  The mandrel-pull
// deformation that retains the rivet is NOT modelled.
//
// Hole-diameter sizing (industry standard, +0.05 mm allowance):
//   hole_dia = nominal_dia + 0.05 mm
//
// Signature pattern:
//   - 1 cylindrical face + 2 circular edges + NO bottom face (through)
//   - pattern.kind == "blind_fastener"
//   - pattern.nominal_dia_mm, pattern.grip_length_mm, pattern.hole_dia_mm
//
// DFM checks:
//   DFM-BF-DIA   : nominal_dia_mm ∈ standard set {3.2, 4.0, 4.8, 6.4}
//   DFM-BF-GRIP  : grip_length_mm within manufacturer's published range
//                  (slice-1 uses generic [0.5, 16] mm bounds per dia)
//   DFM-INPUT    : grip_length_mm > 0
//
// Recognize:
//   History replay first.  Topology fallback flags through cylindrical
//   holes whose diameter matches a standard blind-rivet body size
//   (+0.05 mm install allowance, ± 0.10 mm STEP round-trip tolerance).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace blind_fastener {

constexpr const char* kSkillId = "blind_fastener";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm    = 0.0;
    double      position_y_mm    = 0.0;
    gp_Dir      axis_dir         { 0.0, 0.0, -1.0 };
    double      nominal_dia_mm   = 0.0;
    double      grip_length_mm   = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Helpers exposed for tests.
const std::vector<double>& standardBlindDiameters();             // mm
double holeDiameterFor(double nominal_dia_mm);                   // 0 if unknown
double matchStandardDia(double observed_hole_dia_mm,
                        double tol_mm = 0.10);                   // 0 if no match

// Acceptable grip length range for a given nominal diameter.
struct GripRange { double min_mm; double max_mm; };
GripRange gripRangeFor(double nominal_dia_mm);                   // {0,0} if unknown

}  // namespace blind_fastener
}  // namespace koocadcam::skill
