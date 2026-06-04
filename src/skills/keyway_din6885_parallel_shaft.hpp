#pragma once
// @lat: [[engine/skills#keyway_din6885_parallel_shaft]]
//
// keyway_din6885_parallel_shaft — DIN 6885 Part A parallel-key seat in the
// SHAFT (round-end milled slot per ISO/DIN convention).  Dimensions are
// derived from the central spec table (_keyway_table.hpp); the caller only
// provides shaft diameter, axial position and key length.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace keyway_din6885_parallel_shaft {

constexpr const char* kSkillId = "keyway_din6885_parallel_shaft";

struct Input
{
    FaceDatum face_id;
    // Shaft geometry — assumed centered on +Z.
    gp_Dir    shaft_axis        { 0.0, 0.0, 1.0 };
    double    shaft_center_x_mm = 0.0;
    double    shaft_center_y_mm = 0.0;
    double    shaft_dia_mm      = 0.0;
    // Placement on the shaft.
    double    key_position_z_mm = 0.0;   // axial center of keyway
    double    key_length_mm     = 0.0;
    double    angle_rad         = 0.0;   // angular position of slot center
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace keyway_din6885_parallel_shaft
}  // namespace koocadcam::skill
