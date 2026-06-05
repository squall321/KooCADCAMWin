#pragma once
// @lat: [[engine/skills#mounting_clamp_rail_slot]]
//
// mounting_clamp_rail_slot — SOLAR/PV compound: aluminum-extrusion mounting
// rail T-slot profile for solar-panel mid-clamps and end-clamps.
//
// The T-slot has two stacked rectangular cross-sections:
//
//     ┌─────┐  ← slot_top_width (open mouth)
//     │     │  ← throat region, height = throat_h
//     ├──┬──┤
//     │  │  │  ← throat narrows: slot_throat_width
//     │  │  │
//     │  │  │  (wider hammer-head bay below)
//     └──┴──┘
//
// We extrude the cross-section along the rail axis for the full rail_length.
// Sub-feature chain (SEQUENTIAL pr::cut, 2 ops):
//   1. CUT lower wider bay (hammer-head channel, top_width × bay_h)
//   2. CUT upper narrow throat (throat_width × throat_h) at slot mouth
//
// Signature:
//   pattern.kind                = "mounting_clamp_rail_slot"
//   pattern.is_compound         = true
//   pattern.solar_feature_type  = "t_slot_rail"
//   pattern.subfeature_count    = 2
//   pattern.slot_top_width_mm
//   pattern.slot_throat_width_mm
//   pattern.slot_depth_mm
//
// DFM gates:
//   - all positive
//   - throat_width strictly less than top_width
//   - slot_depth in [4, 30] mm typical for PV rails
//   - rail_length > 0

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace mounting_clamp_rail_slot {

constexpr const char* kSkillId = "mounting_clamp_rail_slot";

struct Input
{
    int    face_id              = -1;
    double rail_axis_origin_x_mm = 0.0;
    double rail_axis_origin_y_mm = 0.0;
    double rail_length_mm        = 100.0;
    double slot_top_width_mm     = 8.0;   // hammer-head bay width
    double slot_throat_width_mm  = 5.0;   // narrower mouth (< top_width)
    double slot_depth_mm         = 12.0;  // total channel depth
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mounting_clamp_rail_slot
}  // namespace koocadcam::skill
