#pragma once
// @lat: [[engine/skills#orfs_face_seal_port]]
//
// orfs_face_seal_port — SAE J1453 O-Ring Face Seal port.
//
// COMPOUND of:
//   1. Straight UN/UNF thread bore at table drill dia (deepest sub-cut).
//   2. Flow bore (slightly larger than the threaded portion) at the mouth.
//   3. Annular face O-ring groove around the bore, on the sealing face.
//   4. Mouth chamfer (light 0.5 × pitch leg) for thread start.
//
// DFM gates:
//   DFM-INPUT      : non-positive sizes.
//   DFM-ORFS-CODE  : orfs_dash not in {-04, -06, -08, -12, -16, -20}.
//   DFM-ORFS-DEPTH : face thickness < depth + 2 mm safety margin.
//
// Recognize: metadata replay (confidence 1.0).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace orfs_face_seal_port {

constexpr const char* kSkillId = "orfs_face_seal_port";

struct Input
{
    FaceDatum   face_id;
    double      position_x_mm = 0.0;
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir     { 0.0, 0.0, -1.0 };
    std::string orfs_dash;                       // "-04" … "-20"
    double      depth_mm = 14.0;                 // total bore depth into part
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace orfs_face_seal_port
}  // namespace koocadcam::skill
