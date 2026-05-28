#pragma once
// @lat: [[engine/skills#ejector_pin_hole]]
//
// ejector_pin_hole — a single straight cylindrical through-hole in a mold
// plate that accepts an ejector pin.  Subtractive feature; axis perpendicular
// to the entry face.
//
//                      ┌─────┐
//                      │     │   ← ejector pin slides through here
//                      │     │
//      ────────────────┴─────┴────────────────  entry_face
//                       dia
//
// Signature pattern:
//   pattern.kind     = "ejector_pin_hole"
//   pattern.dia_mm   = pin diameter
//   pattern.through  = true (always through in slice-1)
//
// DFM:
//   DFM-EJ-DIA       : dia ∈ [1.0, 25.0] mm (standard pin sizes)
//   DFM-EJ-DEPTH     : depth ≥ 2 × dia (must clear)

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace ejector_pin_hole {

constexpr const char* kSkillId = "ejector_pin_hole";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    double      dia_mm        = 0.0;
    double      depth_mm      = 0.0;     // through-cut depth along inward normal
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace ejector_pin_hole
}  // namespace koocadcam::skill
