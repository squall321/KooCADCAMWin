#pragma once
// @lat: [[engine/skills#tab_lock_anti_rotation]]
//
// tab_lock_anti_rotation — COMPOUND MACRO for a tab-lock washer receiver.
//
// Sub-feature chain (3 sub-cuts):
//   1. Notch A on the bore wall   — small rectangular notch radial-in from
//                                   the bore axis at angle 0°, dimensions
//                                   `tab_w` × `tab_d`.  Receives one tab
//                                   of an SAE/AWS tab-lock washer.
//   2. Notch B (diametric)        — same dimensions at angle 180°.  Two
//                                   notches together give an SAE-standard
//                                   biaxial anti-rotation feature.
//   3. Entry chamfer              — 0.5 mm × 45° around the bore rim, guides
//                                   the washer in straight and protects the
//                                   notch corners from peening.
//
//                       ┌── entry_face ──┐
//                       │   ▲ chamfer    │
//                       │  bore mouth    │
//                       │  ▼             │
//                       │  ───┐    ┌───  │   ◄── notch A / B  (at 0° / 180°)
//                       │     │    │     │
//                       │     │    │     │
//                       │  ───┘    └───  │
//                       │                │
//                       └────────────────┘
//
// Standard:
//   - SAE J1199 tab-lock washer typical tab width 1.5–3.0 mm, depth
//     0.5–1.5 mm.
//   - AWS welding-clamp tab geometry differs slightly but the same DFM
//     range applies.
//
// DFM:
//   - tab_w in [0.8, 6.0] mm (mill bit limit and washer geometry).
//   - tab_d in [0.3, 3.0] mm (shallow enough to keep wall thickness).
//   - bore radius ≥ tab_d + 1.0 mm wall (else weakens bore wall).
//
// Signature: is_compound = true, subfeature_count = 3.
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Ax1.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace tab_lock_anti_rotation {

constexpr const char* kSkillId = "tab_lock_anti_rotation";

struct Input
{
    gp_Ax1   bore_axis;                // bore centre line: Location + Direction
    double   bore_dia_mm    = 0.0;     // bore inner diameter (informational)
    double   tab_w_mm       = 0.0;     // notch tangential width
    double   tab_d_mm       = 0.0;     // notch radial depth
    double   tab_axial_z_mm = 0.0;     // notch axial extent (down the bore wall)
    double   chamfer_size_mm = 0.5;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace tab_lock_anti_rotation
}  // namespace koocadcam::skill
