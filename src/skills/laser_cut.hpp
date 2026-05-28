#pragma once
// @lat: [[engine/skills#laser_cut]]
//
// laser_cut — separation cutting with a focused high-power laser beam.
//
// A CO2 or fiber laser melts/vaporises material along a 2D path through
// the stock.  Produces the FINEST kerf and BEST edge quality of the five
// separation processes.  Limited by material thickness (laser power
// drops off sharply beyond 25 mm steel).
//
// Process characteristics:
//   - kerf       : 0.05 – 0.5 mm (very narrow — limited only by spot size)
//   - thickness  : up to 25 mm steel (less for stainless / aluminium / brass)
//   - finish     : "fine" (small HAZ but sharp edge)
//   - tool       : "laser"
//
// Signature pattern:
//   Through-channel with kerf ≈ 0.05 – 0.5 mm; clean vertical walls.
//
// DFM checks:
//   DFM-002       : kerf within process bounds (0.05 – 0.5 mm)
//   DFM-THICKNESS : workpiece Z extent ≤ 25 mm
//   DFM-SURFACE   : info finding noting fine cut-edge finish
//
// Recognize:
//   Channels with kerf < 0.5 mm     → 0.70 (primary — laser is the only
//                                            process capable of < 0.5 mm)
//   Channels with kerf 0.5 – 1.2 mm → 0.30 (secondary — borderline overlap
//                                            with waterjet)

#include "Datum.hpp"
#include "Skill.hpp"
#include "_separation_common.hpp"

#include <array>
#include <memory>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace laser_cut {

constexpr const char* kSkillId = "laser_cut";

using Input = separation_common::LinearCutInput;

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace laser_cut
}  // namespace koocadcam::skill
