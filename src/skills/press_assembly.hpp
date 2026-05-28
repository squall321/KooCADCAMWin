#pragma once
// @lat: [[engine/skills#press_assembly]]
//
// press_assembly — assembly-level interference-fit joint.  Produces a SINGLE
// cylindrical through-hole on this workpiece, slightly undersized so the
// mating pin / shaft (carried by another part) creates a tight press fit
// when assembled.  Conceptually similar to press_fit, but always through and
// always assembly-scoped (records partner_part / press_force_N metadata).
//
//                       ┌─ entry_face (planar)
//                       ▼
//                 ──────┬──────────┬──────
//                       │   ▼ axis │
//                       │  ╱│╲     │
//                       │ ╱ │ ╲    │       (through-hole, slightly
//                       │╱  │  ╲   │        smaller than pin_dia)
//                       │   │   │  │
//                 ──────┴───┴───┴──┴──────
//                            ↑
//                       hole_dia = pin_dia + interference_um × 0.001
//                       (interference_um < 0 → hole < pin)
//
// Signature pattern:
//   - 1 cylindrical face (the bore wall)
//   - 2 circular edges (entry + exit)
//   - NO bottom face (always through)
//   - kind = "press_assembly", carrying pin_dia / interference / press_force_N
//
// DFM checks:
//   DFM-PA-PIN   : pin_diameter_mm ∈ [1.5, 50] mm
//   DFM-PA-INT   : interference_um ∈ [-100, -5] µm     (signed, negative = tight)
//   DFM-PA-FORCE : press_force_N > 0                    (assembly press setup)
//
// Recognize:
//   History replay (confidence 1.0).  Geometric fallback iterates through-
//   cylindrical bores whose diameter is undersize by 5..100 µm relative to a
//   standard pin diameter; confidence 0.50.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace press_assembly {

constexpr const char* kSkillId = "press_assembly";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm    = 0.0;
    double      position_y_mm    = 0.0;
    gp_Dir      axis_dir         { 0.0, 0.0, -1.0 };
    double      pin_diameter_mm  = 0.0;           // mating pin's nominal diameter
    double      interference_um  = -25.0;         // signed; negative = tight
    double      press_force_N    = 0.0;           // calculated assembly force
    std::string partner_part     = "";            // shaft / pin part name
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

// hole_dia_mm = pin_diameter_mm + interference_um × 0.001
double      holeDiameterFor(double pin_diameter_mm, double interference_um);

}  // namespace press_assembly
}  // namespace koocadcam::skill
