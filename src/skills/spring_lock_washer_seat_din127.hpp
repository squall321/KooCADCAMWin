#pragma once
// @lat: [[engine/skills#spring_lock_washer_seat_din127]]
//
// spring_lock_washer_seat_din127 — REAL compound: counterbore seat sized
// for a DIN 127 split-ring spring lock washer.  Counterbore Ø = washer OD +
// 0.4 mm clearance; depth = washer thickness + depth_extra_mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spring_lock_washer_seat_din127 {

constexpr const char* kSkillId      = "spring_lock_washer_seat_din127";
constexpr const char* kRingStandard = "DIN 127";

struct Input
{
    FaceDatum   face_id;                          // planar entry face
    double      center_x_mm        = 0.0;
    double      center_y_mm        = 0.0;
    gp_Dir      axis_dir           { 0.0, 0.0, -1.0 };   // into material
    std::string thread_size_key;                  // "M3" .. "M20"
    double      depth_extra_mm     = 0.5;         // headroom above washer thickness
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace spring_lock_washer_seat_din127
}  // namespace koocadcam::skill
