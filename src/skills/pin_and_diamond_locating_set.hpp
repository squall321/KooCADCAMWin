#pragma once
// @lat: [[engine/skills#pin_and_diamond_locating_set]]
//
// pin_and_diamond_locating_set — the canonical 3-2-1 locating pair.
//
// To locate a flat rectangular part on a fixture you use:
//   • A ROUND pin → kills 2 in-plane DOFs (X and Y at that hole).
//   • A DIAMOND pin (slotted / relieved on two sides) → kills 1 DOF
//     (rotation about the round pin).  The diamond pin is essentially a
//     round pin with two opposing flats so it locates in the perpendicular
//     direction only — slop is absorbed along the round↔diamond line.
//
// In the FIXTURE PLATE (input workpiece) we cut the receptacles for these
// two pins:
//   1. ROUND     receptacle → cylindrical hole at position A
//   2. DIAMOND   receptacle → slot at position B (= round pin Ø + slot
//                              extension along the round↔diamond line).
//   3. CHAMFER   on round receptacle (lead-in)
//   4. CHAMFER   on diamond receptacle (lead-in)
//
// Chain: 4 sub-cuts → subfeature_count = 4.
//
// The diamond receptacle is modelled as a "slotted hole" — a rectangular
// pocket whose end caps are semicircles of round_pin_dia/2.  This matches
// the actual diamond-pin envelope: the flats of the diamond pin clear the
// straight walls, but the curved sides of the pin run against the cylindrical
// end caps so it locates orthogonal to the connection line.
//
// DFM (ASME Y14.5 / ANSI B5.50 / Carr Lane fixture design handbook):
//   • Round pin Ø ≥ 4 mm (smaller pins shear under load)
//   • Connection length L = |B − A| ≥ 3 × round pin Ø (lever arm for
//      angular location)
//   • Slot extension ≥ 0.2 mm and ≤ round pin Ø (else too sloppy)
//   • Hole depth ≥ 1.5 × pin Ø  (engagement)
//   • Chamfer ≥ 0.3 mm (lead-in)

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

namespace koocadcam::skill {

class Workpiece;

namespace pin_and_diamond_locating_set {

constexpr const char* kSkillId = "pin_and_diamond_locating_set";

struct Input
{
    FaceDatum   entry_face;
    gp_Dir      axis_dir          { 0.0, 0.0, -1.0 };   // both pin holes share the same axis
    double      round_x_mm        = 0.0;
    double      round_y_mm        = 0.0;
    double      diamond_x_mm      = 30.0;
    double      diamond_y_mm      = 0.0;
    double      pin_dia_mm        = 6.0;
    double      hole_depth_mm     = 10.0;
    double      diamond_slot_extension_mm = 0.6;   // slot length beyond round (each side)
    double      chamfer_mm        = 0.3;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pin_and_diamond_locating_set
}  // namespace koocadcam::skill
