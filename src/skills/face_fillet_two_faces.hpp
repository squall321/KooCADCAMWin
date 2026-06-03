#pragma once
// @lat: [[engine/skills#face_fillet_two_faces]]
//
// face_fillet_two_faces — SolidWorks-style "Face Fillet" (two faces, constant
// radius).  All edges shared between face_a and face_b are filleted with the
// same radius.  TopExp::MapShapesAndAncestors gives the edge→face map; we
// pick the edges whose ancestor list contains both face_a and face_b.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace face_fillet_two_faces {

constexpr const char* kSkillId = "face_fillet_two_faces";

struct Input
{
    int    face_a_id = -1;
    int    face_b_id = -1;
    double radius_mm = 1.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace face_fillet_two_faces
}  // namespace koocadcam::skill
