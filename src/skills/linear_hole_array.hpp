#pragma once
// @lat: [[engine/skills#linear_hole_array]]
//
// linear_hole_array — GENERATIVE side of the B1.2 grammar's linear pattern.
//
// re::recognizeCompounds recognises a row of equal, evenly-pitched holes as a
// "linear_hole_array" with parametric recovered_params {hole_count, hole_dia_mm,
// pitch_mm, start_x_mm, start_y_mm, direction, axis_dir}.  This skill GENERATES
// that row, so a recognised array becomes a replayable, EDITABLE step — change
// hole_count or pitch_mm and re-execute to regenerate.  Mirrors
// bolt_circle_pattern (drills are composed from drill_hole::apply).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace linear_hole_array {

constexpr const char* kSkillId = "linear_hole_array";

struct Input
{
    FaceDatum entry_face = FaceByNormal{ gp_Dir(0.0, 0.0, 1.0) };
    int       hole_count   = 0;       // >= 3
    double    hole_dia_mm  = 0.0;
    double    start_x_mm   = 0.0;     // first hole centre
    double    start_y_mm   = 0.0;
    double    dir_x        = 1.0;     // in-plane row direction (need not be unit)
    double    dir_y        = 0.0;
    double    pitch_mm     = 0.0;     // centre-to-centre spacing
    gp_Dir    axis_dir     { 0.0, 0.0, -1.0 };
    double    depth_mm     = 0.0;
    bool      through_hole = true;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);   // empty — grammar in koo_re

}  // namespace linear_hole_array
}  // namespace koocadcam::skill
