#pragma once
// @lat: [[engine/skills#oxyfuel_cut]]
//
// oxyfuel_cut — separation cutting with an oxygen-acetylene flame.
//
// A preheating flame brings carbon steel to ignition temperature; a high-
// velocity oxygen jet then oxidises and ejects the steel along a 2D path.
// Used for THICK steel sections (shipyard, structural) where plasma and
// laser cannot reach.  Limited to ferrous materials.
//
// Process characteristics:
//   - kerf       : 3.0 – 10.0 mm (very wide — needs space for oxygen jet)
//   - thickness  : up to 300 mm
//   - finish     : "rough_oxidized" (scale + dross)
//   - tool       : "oxyfuel"
//
// Signature pattern:
//   Through-channel with kerf ≈ 3 – 10 mm; rough oxidised walls.
//
// DFM checks:
//   DFM-002       : kerf within process bounds (3.0 – 10.0 mm)
//   DFM-THICKNESS : workpiece Z extent ≤ 300 mm
//   DFM-SURFACE   : info finding noting rough oxidised edge + dross
//
// Recognize:
//   Channels with kerf > 6.0 mm     → 0.70 (primary)
//   Channels with kerf 3.0 – 6.0 mm → 0.00 (let plasma win, oxyfuel is
//                                            uncommon at this kerf)

#include "Datum.hpp"
#include "Skill.hpp"
#include "_separation_common.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace oxyfuel_cut {

constexpr const char* kSkillId = "oxyfuel_cut";

using Input = separation_common::LinearCutInput;

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace oxyfuel_cut
}  // namespace koocadcam::skill
