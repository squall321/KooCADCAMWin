#pragma once
// @lat: [[engine/skills#cooling_plate_channel_serpentine]]
//
// cooling_plate_channel_serpentine — EV BATTERY compound: liquid-cooling
// serpentine channel cut into the underside of the cell-mounting plate.
//
// Sub-feature chain: an alternating S-pattern of N straight channels (along
// Y axis) connected by 180° turns (half-rings of channel_pitch radius), plus
// a short inlet stub and outlet stub on the closing turns.
//   - straight i ∈ [0, N) :   straight box channel of channel_width × length
//   - turn     j ∈ [0, N-1): semicircular cut connecting straight j to j+1
//   - inlet/outlet plugs:     extension stubs at the start and end straights
//
// Signature:
//   pattern.kind                 = "cooling_plate_channel_serpentine"
//   pattern.is_compound          = true
//   pattern.ev_feature_type      = "cooling_plate_channel"
//   pattern.channel_count        = N
//   pattern.subfeature_count     = N straights + (N-1) turns + 2 stubs
//   pattern.derived_volume_removed_mm3
//
// DFM gates:
//   - all positive
//   - 0.4 ≤ channel_width / channel_depth ≤ 5.0  (manufacturable aspect)
//   - channel_pitch ≥ channel_width + 1.5 (rib wall thickness)
//   - channel_count ≥ 2

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cooling_plate_channel_serpentine {

constexpr const char* kSkillId = "cooling_plate_channel_serpentine";

struct Input
{
    int    face_id           = -1;
    double channel_width_mm  = 6.0;
    double channel_depth_mm  = 3.0;
    double channel_pitch_mm  = 12.0;     // centerline-to-centerline of straights
    int    channel_count     = 4;
    double straight_length_mm = 80.0;    // length of each straight
    double inlet_x_mm        = 0.0;      // start of first straight
    double inlet_y_mm        = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace cooling_plate_channel_serpentine
}  // namespace koocadcam::skill
