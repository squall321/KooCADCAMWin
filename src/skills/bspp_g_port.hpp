#pragma once
// @lat: [[engine/skills#bspp_g_port]]
//
// bspp_g_port — BSPP / ISO 228-1 G-thread hydraulic port.
//
// COMPOUND of:
//   1. Main parallel-thread bore at the tap-drill diameter.
//   2. Entry chamfer (45° / leg = 0.5 × thread pitch from central table).
//   3. Small face counterbore relief at the mouth (tool relief, 0.5 × pitch
//      deeper than the chamfer leg).
//
// DFM gates:
//   DFM-INPUT      : non-positive sizes.
//   DFM-BSPP-CODE  : g_size not in {G1/8 … G1}.
//   DFM-BSPP-DEPTH : face thickness < depth + 2 mm safety margin.
//
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bspp_g_port {

constexpr const char* kSkillId = "bspp_g_port";

struct Input
{
    FaceDatum   face_id;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };
    std::string g_size;                          // "G1/8" … "G1"
    double      depth_mm = 12.0;                 // bore depth into part
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bspp_g_port
}  // namespace koocadcam::skill
