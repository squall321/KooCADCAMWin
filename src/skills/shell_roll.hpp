#pragma once
// @lat: [[engine/skills#shell_roll]]
//
// shell_roll — roll a rectangular plate into a cylindrical pressure-vessel
// shell using a 3- or 4-roll plate-bending machine.  The plate enters flat;
// successive passes increase curvature until the two long edges meet, ready
// for the longitudinal seam weld.  This is the standard first step in
// fabricating any shell-and-tube heat exchanger or pressure vessel.
//
//      ┌──────────────────────────────────┐                  ╭─────────╮
//      │  flat plate                       │   3-roll bend   │         │
//      │  L × π·D (developed length)       │  ─────────────► │  shell  │
//      │  thickness = t                    │                 │   D     │
//      └──────────────────────────────────┘                  ╰─────────╯
//                                                            length = L
//
// Signature pattern:
//   - pattern.kind             = "shell_roll"
//   - pattern.plate_thick_mm
//   - pattern.shell_dia_mm
//   - pattern.shell_length_mm
//   - pattern.developed_length_mm = π × dia (verification)
//   - pattern.D_over_t            (slenderness ratio for downstream DFM)
//   - pattern.geometry_changed    = false
//
// DFM checks:
//   DFM-INPUT             any dimension ≤ 0
//   DFM-ROLL-THK-MIN      plate_thick_mm < 3.0 → buckling risk (warning)
//   DFM-ROLL-THK-MAX      plate_thick_mm > 80.0 → cold-roll not feasible,
//                         hot-form required (info)
//   DFM-ROLL-D-OVER-T     shell_dia_mm / plate_thick_mm < 10 → tight-radius
//                         bend, may need pre-heat (info)
//
// Synthesis:
//   Geometric no-op for slice-1.  Pre-roll plate and post-roll cylinder are
//   modelled at the assembly level, not in this skill.
//
// Recognize:
//   Metadata replay.

#include "Skill.hpp"

#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace shell_roll {

constexpr const char* kSkillId = "shell_roll";

struct Input
{
    double  plate_thick_mm  = 12.0;
    double  shell_dia_mm    = 600.0;
    double  shell_length_mm = 3000.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace shell_roll
}  // namespace koocadcam::skill
