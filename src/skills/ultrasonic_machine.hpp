#pragma once
// @lat: [[engine/skills#ultrasonic_machine]]
//
// ultrasonic_machine (USM) — Ultrasonic Machining via abrasive slurry.
//
// A shaped tool vibrating at 20–40 kHz drives an abrasive (B4C / SiC / Al2O3)
// slurry against the workpiece.  The micro-impact of the abrasive particles
// chips the workpiece in the shape of the tool's negative.  USM is the
// process of choice for HARD-AND-BRITTLE materials that cannot be machined
// by conventional cutting or by EDM (no electrical conductivity required):
//
//   - glass (soda-lime, borosilicate, fused silica)
//   - ceramics (alumina, zirconia, silicon nitride)
//   - silicon wafers
//   - cemented carbides
//   - precious stones (ruby, sapphire bearings)
//
// Geometry: square OR round pocket — same shape family as a milled pocket
// or sinker EDM cavity.  The hallmark USM constraint is the MATERIAL
// CLASS: USM only works on brittle materials.  We INFO-level flag any
// material not in the brittle catalog (does not reject — the heat-
// treatment skills set the precedent that material gates are advisory).
//
//                       entry_face (planar)
//                            │
//                ────────────┼──────────────
//                            │     ↓ axis
//                  ┌─────────┴─────────┐   ↑ depth_mm
//                  │  (square or       │   │
//                  │   round pocket)   │   │
//                  └───────────────────┘   ↓
//
// Signature pattern:
//   shape = "square"  → 4 vertical walls + 1 flat bottom
//   shape = "round"   → 1 cylindrical wall + 1 flat bottom
//
// DFM checks:
//   DFM-INPUT           shape ∉ {"square","round"}, depth ≤ 0, dim ≤ 0
//   DFM-USM-MATERIAL    material not in brittle-catalog → info
//   DFM-USM-DEPTH       depth > 10 mm → warning (abrasive flow stagnation)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ultrasonic_machine {

constexpr const char* kSkillId = "ultrasonic_machine";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    std::string shape;                  // "square" | "round"
    double      length_mm      = 0.0;   // for "square"
    double      width_mm       = 0.0;   // for "square"
    double      diameter_mm    = 0.0;   // for "round"
    double      depth_mm       = 0.0;
    std::string material       = "";    // optional override; default: wp.material()
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ultrasonic_machine
}  // namespace koocadcam::skill
