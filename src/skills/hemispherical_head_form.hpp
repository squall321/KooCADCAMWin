#pragma once
// @lat: [[engine/skills#hemispherical_head_form]]
//
// hemispherical_head_form — form a hemispherical end closure for a pressure
// vessel from a circular plate blank, typically by hot or cold spinning,
// pressing, or hydro-forming.  Hemispherical heads are the most weight-
// efficient closure shape for high-pressure service because the stress is
// uniformly distributed in two principal directions.
//
//                              ╭───────────╮
//                              │           │       ╭─────╮
//        ┌─────────────┐       │           │      ╱       ╲
//        │             │       │           │     │  hemi   │
//        │  flat blank │  ──►  │  drawn /  │ ──► │  head    │
//        │  D_blank    │       │  pressed  │      ╲       ╱
//        │  thick t    │       │           │       ╰─────╯
//        └─────────────┘       │           │       dia = D
//                              ╰───────────╯
//
// Blank diameter rule of thumb:  D_blank ≈ 1.27 × D_head (for spinning).
//
// Signature pattern:
//   - pattern.kind             = "hemispherical_head_form"
//   - pattern.dia_mm
//   - pattern.plate_thick_mm
//   - pattern.crown_radius_mm  = dia_mm / 2 (true hemisphere)
//   - pattern.t_over_D         (thickness / diameter ratio)
//   - pattern.geometry_changed = false
//
// DFM checks:
//   DFM-INPUT          dia_mm ≤ 0  or  plate_thick_mm ≤ 0
//   DFM-HEAD-T-OVER-D  t / D < 0.005 → thin-shell buckling risk (warning)
//   DFM-HEAD-T-OVER-D  t / D > 0.25  → forging-class section, hemi-spin not
//                                       practical (info)
//
// Synthesis:
//   Geometric no-op for slice-1; head is modelled at assembly level.
//
// Recognize:
//   Metadata replay only.

#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hemispherical_head_form {

constexpr const char* kSkillId = "hemispherical_head_form";

struct Input
{
    double  dia_mm         = 600.0;
    double  plate_thick_mm = 12.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hemispherical_head_form
}  // namespace koocadcam::skill
