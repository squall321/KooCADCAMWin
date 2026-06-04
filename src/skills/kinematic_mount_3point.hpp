#pragma once
// @lat: [[engine/skills#kinematic_mount_3point]]
//
// kinematic_mount_3point — OPTICAL compound: three-point kinematic mount
// (sphere / V-groove / flat seats — classic Maxwell layout).
//
// Sub-features (3 ops):
//   1. cut hemispherical pocket at seat 0      (CUT)
//   2. cut V-groove slot at seat 1             (CUT)
//   3. cut shallow flat circular pad at seat 2 (CUT)
//
// Seat kinds (legacy override): the standard layout puts one of each at each
// of the 3 seats.  `seat_kind` selects the "primary" seat type to label.
//
// DFM:
//   DFM-INPUT      : face_id valid, sphere_dia_mm > 0
//   DFM-SEATS      : 3 seat positions provided + distinct from each other
//   DFM-SEAT-KIND  : seat_kind ∈ {sphere_pocket, v_groove, flat_pad}

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>

namespace koocadcam::skill {

class Workpiece;

namespace kinematic_mount_3point {

constexpr const char* kSkillId = "kinematic_mount_3point";

struct Input
{
    int         face_id        = -1;
    double      seat0_x_mm     = 0.0;
    double      seat0_y_mm     = 0.0;
    double      seat1_x_mm     = 20.0;
    double      seat1_y_mm     = 0.0;
    double      seat2_x_mm     = 10.0;
    double      seat2_y_mm     = 17.32;   // equilateral triangle
    std::string seat_kind      = "sphere_pocket";  // primary seat label
    double      sphere_dia_mm  = 6.35;    // 1/4" ball
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace kinematic_mount_3point
}  // namespace koocadcam::skill
