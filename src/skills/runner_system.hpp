#pragma once
// @lat: [[engine/skills#runner_system]]
//
// runner_system — array of small cylindrical channels carved through a mold
// plate that distribute molten plastic from the sprue to individual cavity
// gates.  Modelled as a SUBTRACTIVE feature: each (start, end) point pair
// defines one straight cylindrical channel of diameter `dia_mm` cut through
// the workpiece.
//
//                       (start0)──── runner0 ────(end0)
//                                                   │
//                       (start1)──── runner1 ────(end1)
//                                                   │
//
// Signature pattern:
//   pattern.kind          = "runner_system"
//   pattern.dia_mm        = channel diameter
//   pattern.segment_count = number of runner segments
//
// DFM:
//   DFM-RUNNER-DIA   : dia ∈ [2.0, 8.0] mm (typical range)
//   DFM-RUNNER-LEN   : each segment length ≥ 3 × dia
//   DFM-RUNNER-N     : at least one segment required

#include "Datum.hpp"
#include "Skill.hpp"

#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace runner_system {

constexpr const char* kSkillId = "runner_system";

// A single runner segment in world coordinates (3D start/end of the channel
// axis).  Endpoints define the FULL through-cut along that line — the cutter
// is a cylinder whose length matches |end - start|.
struct Segment
{
    double start_x_mm = 0.0, start_y_mm = 0.0, start_z_mm = 0.0;
    double end_x_mm   = 0.0, end_y_mm   = 0.0, end_z_mm   = 0.0;
};

struct Input
{
    std::vector<Segment>  segments;
    double                dia_mm = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace runner_system
}  // namespace koocadcam::skill
