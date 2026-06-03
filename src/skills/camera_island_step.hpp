#pragma once
// @lat: [[engine/skills#camera_island_step]]
//
// camera_island_step — PHONE compound: camera bump (stepped raised feature)
// with a chamfered ledge and circular lens recess.
//
// Sub-features (4 ops):
//   1. fuse rectangular step (height = raised_height) onto face (FUSE)
//   2. fuse a thin chamfer ledge wider than the step                     (FUSE)
//   3. cut circular lens recess into the raised top                      (CUT)
//   4. cut a chamfer relief at the outer rectangular edge                (CUT)
//
// DFM:
//   DFM-INPUT      : all dims > 0
//   DFM-CAM-LENS   : inner_lens_dia_mm < outer_island_w_mm (lens fits)
//   DFM-CAM-LENS   : inner_lens_depth_mm ≤ raised_height_mm

#include "Datum.hpp"
#include "Skill.hpp"

namespace koocadcam::skill {

class Workpiece;

namespace camera_island_step {

constexpr const char* kSkillId = "camera_island_step";

struct Input
{
    int    face_id              = -1;
    double camera_center_x_mm   = 0.0;
    double camera_center_y_mm   = 0.0;
    double outer_island_w_mm    = 20.0;  // X
    double outer_island_h_mm    = 10.0;  // Y
    double raised_height_mm     = 1.5;
    double inner_lens_dia_mm    = 6.0;
    double inner_lens_depth_mm  = 0.8;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace camera_island_step
}  // namespace koocadcam::skill
