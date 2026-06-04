#pragma once
// @lat: [[engine/skills#hex_drive_recess_implant]]
//
// hex_drive_recess_implant — internal hex socket recess for implant
// driver instruments (hex-A/F).  A pure hexagonal-prism cut, oriented so
// two flats face ±X.
//
// Standards reference:
//   ISO 4762 / DIN 912           Hex socket cap screw (industrial standard)
//   Medical/dental hex-driver convention — across-flats sizes
//     1.5, 2.0, 2.5, 3.0, 4.0, 5.0 mm
//
// Hex circumradius (corner) = AF / √3.
//
// DFM:
//   DFM-HEX-AF      : hex_AF_mm ∉ {1.5, 2.0, 2.5, 3.0, 4.0, 5.0}
//   DFM-HEX-DEPTH   : depth_mm ≤ 0 or > stock thickness
//
// signature.pattern.kind                  = "hex_drive_recess_implant"
// signature.pattern.is_compound           = true
// signature.pattern.medical_feature_type  = "implant_hex_driver_socket"

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace hex_drive_recess_implant {

constexpr const char* kSkillId = "hex_drive_recess_implant";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    double      hex_AF_mm      = 0.0;   // 1.5 .. 5.0 (across-flats)
    double      depth_mm       = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace hex_drive_recess_implant
}  // namespace koocadcam::skill
