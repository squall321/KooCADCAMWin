#pragma once
// @lat: [[engine/skills#keyway_internal_din6885_through]]
//
// keyway_internal_din6885_through — broached internal keyway running the
// FULL length of a hub bore (DIN 6885 Part A geometry; through-cut).
// The shaft is unspecified — the cutter runs the entire workpiece Z-extent.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace keyway_internal_din6885_through {

constexpr const char* kSkillId = "keyway_internal_din6885_through";

struct Input
{
    FaceDatum bore_face_id;
    gp_Dir    bore_axis        { 0.0, 0.0, 1.0 };
    double    bore_center_x_mm = 0.0;
    double    bore_center_y_mm = 0.0;
    double    bore_dia_mm      = 0.0;
    double    angle_rad        = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace keyway_internal_din6885_through
}  // namespace koocadcam::skill
