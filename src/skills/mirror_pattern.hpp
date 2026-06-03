#pragma once
// @lat: [[engine/skills#mirror_pattern]]
//
// mirror_pattern — duplicate a single hole across a plane through origin.
//
// mirror_axis = "x" → mirror across the YZ plane (x → -x).
// mirror_axis = "y" → mirror across the XZ plane (y → -y).
//
// apply():  drill the original (hole_x_mm, hole_y_mm) + drill the mirrored
//           position via two pr::cylinder cuts.
//
// DFM gates:
//   DFM-INPUT  : hole_dia_mm > 0, hole_depth_mm > 0
//   DFM-002    : hole_dia_mm ≥ 0.8 mm
//   DFM-INPUT  : mirror_axis must be "x" or "y"
//   DFM-PATT-3 : original and mirror positions must not overlap (distance
//                between them > hole_dia_mm)

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>

namespace koocadcam::skill {

class Workpiece;

namespace mirror_pattern {

constexpr const char* kSkillId = "mirror_pattern";

struct Input
{
    double      hole_x_mm     = 0.0;
    double      hole_y_mm     = 0.0;
    double      hole_dia_mm   = 0.0;
    double      hole_depth_mm = 0.0;
    FaceDatum   face;                       // host face
    std::string mirror_axis   = "x";        // "x" or "y"
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace mirror_pattern
}  // namespace koocadcam::skill
