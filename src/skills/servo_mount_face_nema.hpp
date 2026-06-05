#pragma once
// @lat: [[engine/skills#servo_mount_face_nema]]
//
// servo_mount_face_nema — NEMA stepper/servo mounting face
// (slice 16, robotics / automation).
//
// A motor-mount plate that accepts a NEMA-frame stepper or servo: a central
// pilot boss bore that registers the motor's spigot, surrounded by four
// corner bolt holes on the NEMA square pattern.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. central pilot boss bore (cylinder cut)        — biggest feature first
//   2..5. 4 corner bolt holes on the NEMA bolt square
//
// subfeature_count = 1 + 4.
//
// NEMA bolt-square pitch (corner-to-corner, mm): 17=31.0, 23=47.14, 34=69.6.
// NEMA pilot diameter (mm):                       17=22,   23=38.1, 34=73.
//
// DFM:
//   DFM-INPUT       : pilot_dia_mm must be > 0.
//   DFM-NEMA        : nema_size must be "17" | "23" | "34".
//   DFM-THREAD      : bolt_thread_key must exist in central metric table.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace servo_mount_face_nema {

constexpr const char* kSkillId = "servo_mount_face_nema";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy        { 0.0, 0.0, 0.0 };
    std::string nema_size         = "23";   // "17" | "23" | "34"
    double      pilot_dia_mm      = 38.1;
    std::string bolt_thread_key   = "M5";   // from _iso_thread_table.hpp
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace servo_mount_face_nema
}  // namespace koocadcam::skill
