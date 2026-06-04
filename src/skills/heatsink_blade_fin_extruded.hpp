#pragma once
// @lat: [[engine/skills#heatsink_blade_fin_extruded]]
//
// heatsink_blade_fin_extruded — COMPOUND extruded blade-fin heat sink.
//
// Adds a rectangular base plate + N upright rectangular blade fins on top.
// The fins run along +X (fin_length axis); they are pitched along +Y.  This
// is the most common machined / extruded heat-sink topology for power
// electronics enclosures (LDOs, drivers, motor inverters).
//
// Sub-feature chain:
//   build_base_plate (optional, only if base_thickness_mm > 0)
//   build_fin_box × N
//   fuseMany onto wp
//
// DFM gates:
//   DFM-FIN-THK : fin_thickness_mm < 0.5 mm (cutter breakage / sheet too thin).
//   DFM-FIN-AR  : fin_height / fin_thickness > 12 (manufacturable AR cap).
//   DFM-FIN-N   : fin_count < 2 (single rib, not an array).
//   DFM-FIN-PITCH: fin_pitch <= fin_thickness (no air gap).
//   DFM-FIN-LEN : fin_length_mm <= 0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace heatsink_blade_fin_extruded {

constexpr const char* kSkillId = "heatsink_blade_fin_extruded";

struct Input
{
    int    face_id            = -1;
    double base_thickness_mm  = 0.0;   // 0 → no base plate, fins direct on wp
    int    fin_count          = 0;
    double fin_pitch_mm       = 0.0;
    double fin_thickness_mm   = 0.0;
    double fin_height_mm      = 0.0;
    double fin_length_mm      = 0.0;
    // First fin's left-back corner (XY) on the base face plane.
    double origin_x_mm        = 0.0;
    double origin_y_mm        = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace heatsink_blade_fin_extruded
}  // namespace koocadcam::skill
