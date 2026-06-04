#pragma once
// @lat: [[engine/skills#keyway_din6886_taper_shaft]]
//
// keyway_din6886_taper_shaft — DIN 6886 taper key seat in a shaft.
// The key is rectangular in cross section, b × h at the thick end, tapered
// 1:100 along its length.  The seat in the shaft is the negative of the
// key's bottom face, mirroring the 1:100 taper.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace keyway_din6886_taper_shaft {

constexpr const char* kSkillId = "keyway_din6886_taper_shaft";

struct Input
{
    FaceDatum face_id;
    gp_Dir    shaft_axis        { 0.0, 0.0, 1.0 };
    double    shaft_center_x_mm = 0.0;
    double    shaft_center_y_mm = 0.0;
    double    shaft_dia_mm      = 0.0;
    double    key_position_z_mm = 0.0;
    double    key_length_mm     = 0.0;
    double    angle_rad         = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace keyway_din6886_taper_shaft
}  // namespace koocadcam::skill
