#pragma once
// @lat: [[engine/skills#pipe_clamp_saddle_compound]]
//
// pipe_clamp_saddle_compound — saddle pipe clamp with 2 bolt holes (HVAC).
//
// Slice 15 — HVAC/fluid.
//
// Sub-features (sequential pr::cut, no compound boolean):
//   1. saddle pocket: a half-cylinder cut matching the pipe OD + clearance,
//      sized to seat the pipe on top of the bracket.
//   2. bolt A clearance hole (full-through, M-thread clearance from central table)
//   3. bolt B clearance hole (full-through, M-thread clearance from central table)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-M-THREAD    : bolt_thread_size_key not in central metric thread table.
//   DFM-PIPE-DIA    : pipe_dia_mm <= 0.
//   DFM-SADDLE-THK  : saddle_thickness < 0.5 × pipe_dia (insufficient saddle wall).
//   DFM-BOLT-SPACE  : bolts too close to saddle bore (overlap).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pipe_clamp_saddle_compound {

constexpr const char* kSkillId = "pipe_clamp_saddle_compound";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      pipe_axis_origin   { 0.0, 0.0, 0.0 };
    gp_Dir      pipe_axis_dir      { 1.0, 0.0, 0.0 };   // pipe along +X
    double      pipe_dia_mm        = 25.0;
    double      saddle_thickness_mm = 6.0;
    double      bolt_a_xy[2]       = { -20.0, 0.0 };   // XY of bolt A on top face
    double      bolt_b_xy[2]       = {  20.0, 0.0 };
    std::string bolt_thread_size_key;                  // e.g. "M8"
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pipe_clamp_saddle_compound
}  // namespace koocadcam::skill
