#pragma once
// @lat: [[engine/skills#plasma_cut]]
//
// plasma_cut — separation cutting with an ionised gas (plasma) arc.
//
// A high-temperature plasma jet ionises and melts conductive material
// (steel, aluminium, stainless) along a 2D path.  FAST on thick conductors;
// kerf is wider than laser/waterjet but THINNER than oxyfuel.
//
// Process characteristics:
//   - kerf       : 1.0 – 6.0 mm (depends on torch / current)
//   - thickness  : up to 50 mm
//   - finish     : "rough_oxidized" (heat-affected, scaled edge)
//   - tool       : "plasma"
//
// Signature pattern:
//   Through-channel with kerf ≈ 1 – 6 mm; rough oxidised vertical walls.
//
// DFM checks:
//   DFM-002       : kerf within process bounds (1.0 – 6.0 mm)
//   DFM-THICKNESS : workpiece Z extent ≤ 50 mm
//   DFM-SURFACE   : info finding noting rough oxidised edge
//
// Recognize:
//   Channels with kerf 3.0 – 6.0 mm → 0.60 (primary)
//   Channels with kerf 1.2 – 3.0 mm → 0.30 (also plausible — overlap with saw)
//   Channels with kerf > 6.0 mm     → 0.30 (could also be oxyfuel)

#include "Datum.hpp"
#include "Skill.hpp"
#include "_separation_common.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace plasma_cut {

constexpr const char* kSkillId = "plasma_cut";

using Input = separation_common::LinearCutInput;

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace plasma_cut
}  // namespace koocadcam::skill
