#pragma once
// @lat: [[engine/skills#rivet_hole]]
//
// rivet_hole — through-hole specifically sized & toleranced for a rivet.
// Same B-rep geometry as drill_hole(through_hole=true) but with tighter
// tolerance metadata + standard-class lookup (M5 / 3/16" / etc.).
//
//                       ┌─ entry_face (planar)
//                       ▼
//                 ──────┬──────────┬──────
//                       │   ▼ axis │
//                       │  ╱│╲     │
//                       │ ╱ │ ╲    │       (through hole — no bottom)
//                       │╱  │  ╲   │
//                       │   │   │  │
//                 ──────┴───┴───┴──┴──────
//                            ↑
//                          rivet_dia
//
// Signature pattern:
//   - 1 cylindrical face (the bore wall)
//   - 2 circular edges (entry & exit), no bottom face
//   - rivet_diameter_class string carries the spec (e.g. "M5", "ANSI_3_16in")
//
// DFM checks:
//   rivet_dia ≥ 2.0 mm     (smallest standard rivet)
//   through always true    (a blind hole would not pass a rivet shank)
//   depth/dia ≤ 8.0        (peck warning, same as drill_hole)
//
// Recognize:
//   Through cylindrical face with diameter matching a STANDARD rivet size:
//     {1.6, 2.0, 2.4, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0} mm
//   Tolerance ± 0.15 mm for STEP round-trip noise.  Confidence high if the
//   nearest standard is within 0.05 mm; low if 0.05 – 0.15.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rivet_hole {

constexpr const char* kSkillId = "rivet_hole";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir      { 0.0, 0.0, -1.0 };
    double      diameter_mm   = 0.0;       // typ. matches a standard rivet
    std::string rivet_diameter_class;      // "M5" | "ANSI_3_16in" | …
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// Helpers exposed for tests and recognition.
const std::vector<double>& standardRivetDiameters();
std::string                 classFromDiameter(double dia_mm);   // returns "" if none
double                      diameterFromClass(const std::string& cls);  // 0.0 if unknown

}  // namespace rivet_hole
}  // namespace koocadcam::skill
