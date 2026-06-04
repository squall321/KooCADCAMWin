#pragma once
// @lat: [[engine/skills#keyway_woodruff_arc]]
//
// keyway_woodruff_arc — semi-circular arc keyway for a Woodruff key
// (DIN 6888 / ANSI B17.2).  The seat is cut by a cylindrical "Woodruff
// cutter" rolled into the shaft surface; the resulting feature is an arc
// trough.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace keyway_woodruff_arc {

constexpr const char* kSkillId = "keyway_woodruff_arc";

struct Input
{
    FaceDatum   face_id;
    gp_Dir      shaft_axis        { 0.0, 0.0, 1.0 };
    double      shaft_center_x_mm = 0.0;
    double      shaft_center_y_mm = 0.0;
    double      shaft_dia_mm      = 0.0;
    double      key_position_z_mm = 0.0;
    double      angle_rad         = 0.0;
    std::string woodruff_size_id;        // "404", "808", "1212", ...
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace keyway_woodruff_arc
}  // namespace koocadcam::skill
