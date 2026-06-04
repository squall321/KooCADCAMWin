#pragma once
// @lat: [[engine/skills#keyway_din6885_parallel_hub]]
//
// keyway_din6885_parallel_hub — DIN 6885 Part A parallel-key seat in a HUB
// bore (broached slot, square corners, full bore length OR specified
// length).  Dimensions sourced from the central DIN 6885 spec table.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace keyway_din6885_parallel_hub {

constexpr const char* kSkillId = "keyway_din6885_parallel_hub";

struct Input
{
    FaceDatum bore_face_id;
    gp_Dir    bore_axis        { 0.0, 0.0, 1.0 };
    double    bore_center_x_mm = 0.0;
    double    bore_center_y_mm = 0.0;
    double    bore_dia_mm      = 0.0;
    double    key_position_z_mm = 0.0;
    double    key_length_mm    = 0.0;
    double    angle_rad        = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace keyway_din6885_parallel_hub
}  // namespace koocadcam::skill
