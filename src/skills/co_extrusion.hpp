#pragma once
// @lat: [[engine/skills#co_extrusion]]
//
// co_extrusion — continuous extrusion of a multi-layer profile.  Examples:
// weatherstrip with rigid carrier + soft sealing lip; multi-layer pipe with
// barrier core; co-ex sheet for thermoforming with skin/foam/skin.
//
//          ┌── layer N: outer skin (material_N, thickness_N) ──┐
//          ├── layer 2: barrier    (material_2, thickness_2) ──┤
//          ├── layer 1: core       (material_1, thickness_1) ──┤
//          │                  …                                 │
//          └────────────────────────────────────────────────────┘
//                     extrusion direction →
//
// METADATA-ONLY.  The workpiece is treated as the finished extruded profile
// at the time the skill is applied; co_extrusion records the layer stack so
// downstream consumers can run flow-balance / die-design analyses.
//
// Signature pattern:
//   pattern.kind             == "co_extrusion"
//   pattern.layer_count      == layers.size()
//   pattern.layers           == [{material, thickness_mm}, ...]
//   pattern.total_thickness_mm
//   pattern.geometric_change == false
//
// DFM:
//   DFM-COEX-COUNT      : layer_count ∈ [2, 7]
//   DFM-COEX-THICKNESS  : each thickness > 0 ; sum > 0
//   DFM-COEX-MATERIAL   : every layer has a non-empty material id

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace co_extrusion {

struct CoExLayer
{
    std::string material      = "abs";
    double      thickness_mm  = 0.5;
};

constexpr const char* kSkillId = "co_extrusion";

struct Input
{
    std::vector<CoExLayer> layers;     // ordered from inner → outer
};

SkillOutput                       apply   (const Workpiece& wp, const Input& in);
DFMReport                         validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature>    recognize(const Workpiece& wp);

}  // namespace co_extrusion
}  // namespace koocadcam::skill
