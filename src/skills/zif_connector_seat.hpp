#pragma once
// @lat: [[engine/skills#zif_connector_seat]]
//
// zif_connector_seat — PHONE compound: Zero-Insertion-Force connector pocket
// for flex cables (FPC).
//
// Sub-features (3 ops):
//   1. narrow main pocket               (cut)  (fpc_width + tol) × H × W
//   2. wider clearance pocket above     (cut)  for the latch hinge swing
//   3. lead-in chamfer at FPC entry     (cut)  thin slope to help insertion
//
// DFM:
//   DFM-INPUT             : all dims > 0; latch_clearance_mm ≥ 0
//   DFM-ZIF-RANGE         : fpc_width_mm ∈ [1.5, 15]; height ∈ [0.5, 1.2]

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace zif_connector_seat {

constexpr const char* kSkillId = "zif_connector_seat";

struct Input
{
    int    face_id              = -1;
    double position_x_mm        = 0.0;
    double position_y_mm        = 0.0;
    double fpc_width_mm         = 6.0;  // FPC ribbon width
    double connector_height_mm  = 0.8;  // pocket depth below face
    double latch_clearance_mm   = 0.4;  // extra clearance above for hinge
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace zif_connector_seat
}  // namespace koocadcam::skill
