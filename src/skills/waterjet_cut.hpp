#pragma once
// @lat: [[engine/skills#waterjet_cut]]
//
// waterjet_cut — separation cutting with a high-pressure abrasive water jet.
//
// A focused ~0.3-mm-diameter jet of water mixed with abrasive (typically
// garnet) traverses a 2D path through the stock.  Produces a smooth
// cut-edge with no heat-affected zone — ideal for delicate materials
// (composites, glass, thick stainless) where laser HAZ is unacceptable.
//
// Process characteristics:
//   - kerf       : 0.6 – 1.2 mm (abrasive stream is wider than the water jet)
//   - thickness  : up to 200 mm (water jet has no upper material limit)
//   - finish     : "smooth" (no thermal damage)
//   - tool       : "waterjet"
//
// Signature pattern:
//   Through-channel with kerf ≈ 0.6 – 1.2 mm; smooth vertical walls.
//
// DFM checks:
//   DFM-002       : kerf within process bounds (0.6 – 1.2 mm)
//   DFM-THICKNESS : workpiece Z extent ≤ 200 mm
//   DFM-SURFACE   : info finding noting smooth cut-edge finish
//
// Recognize:
//   Channels with kerf 0.5 – 1.2 mm → 0.70 (primary)
//   Channels with kerf 1.2 – 3.0 mm → 0.50 (secondary — saw is more common)
//   Channels with kerf < 0.5 mm     → 0.50 (could also be laser)

#include "Datum.hpp"
#include "Skill.hpp"
#include "_separation_common.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace waterjet_cut {

constexpr const char* kSkillId = "waterjet_cut";

using Input = separation_common::LinearCutInput;

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace waterjet_cut
}  // namespace koocadcam::skill
