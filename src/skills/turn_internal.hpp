#pragma once
// @lat: [[engine/skills#turn_internal]]
//
// turn_internal — internal-diameter (ID) boring/turning operation.
//
// Enlarges an existing bore over a Z range: the bore's radius goes from
// existing_bore_dia_mm/2 to final_dia_mm/2 (with final_dia > existing_bore_dia).
//
//     │           │   │           │
//     │  bore     │   │           │
//     │ existing  │   │ enlarged  │
//     │   dia     │ → │   final   │
//     │           │   │   dia     │
//     │           │   │           │
//     │           │   │           │
//
// Geometrically we cut a cylinder of radius final_dia/2 over the Z range from
// the workpiece.  The result has:
//
//   - 1 NEW cylindrical face at radius final_dia/2 (the enlarged section)
//   - 1 NEW annular planar shoulder face at end_z (normal +Z) where the
//     enlarged bore meets the still-narrow original bore above.
//   - 1 NEW annular planar shoulder face at start_z (normal -Z) if the
//     existing bore is blind, OR break-through into bottom.
//
// Signature pattern:
//   - 2 coaxial cylindrical faces with DIFFERENT radii (the new enlarged
//     section + the remaining original-diameter section above/below).
//   - 1 annular planar shoulder face at the meeting plane.
//
// DFM checks:
//   DFM-INPUT  : final_dia > existing_bore_dia, both ≥ 0.8 mm
//   DFM-INPUT  : end_z > start_z

#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace turn_internal {

constexpr const char* kSkillId = "turn_internal";

struct Input
{
    double  start_z_mm           = 0.0;
    double  end_z_mm             = 0.0;
    double  final_dia_mm         = 0.0;   // enlarged bore diameter
    double  existing_bore_dia_mm = 0.0;   // bore diameter before this op
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace turn_internal
}  // namespace koocadcam::skill
