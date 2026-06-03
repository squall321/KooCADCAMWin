#pragma once
// @lat: [[engine/skills#jic_37deg_flare_port]]
//
// jic_37deg_flare_port — SAE J514 / ISO 8434-2 JIC 37° flare port.
//
// COMPOUND of:
//   1. UNF straight-thread bore (drill size derived from table; tapped after).
//   2. Conical 37° flare seat (half-angle, included = 74°) at the mouth.
//   3. Inner flow-passage bore (smaller dia, deeper than the flare seat).
//
// DFM gates:
//   DFM-INPUT     : non-positive sizes.
//   DFM-JIC-CODE  : jic_dash not in {-04, -06, -08, -12}.
//   DFM-JIC-DEPTH : face thickness < depth + 2 mm safety margin.
//
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace jic_37deg_flare_port {

constexpr const char* kSkillId = "jic_37deg_flare_port";

struct Input
{
    FaceDatum   face_id;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };
    std::string jic_dash;                        // "-04" … "-12"
    double      depth_mm = 14.0;                 // total bore depth into part
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace jic_37deg_flare_port
}  // namespace koocadcam::skill
