#pragma once
// @lat: [[engine/skills#rectangular_hole_grid]]
//
// rectangular_hole_grid — GENERATIVE side of the B1.2 grammar's grid pattern
// (e.g. a watch speaker grille).  re::recognizeCompounds recognises a filled
// cols×rows lattice of equal holes and (since the grammar enrichment) records
// its origin + unit basis directions, so this skill can regenerate it: change
// cols/rows/pitch and re-execute.  Mirrors bolt_circle_pattern / linear_hole_array
// (drills are composed from drill_hole::apply).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace rectangular_hole_grid {

constexpr const char* kSkillId = "rectangular_hole_grid";

struct Input
{
    FaceDatum entry_face = FaceByNormal{ gp_Dir(0.0, 0.0, 1.0) };
    int       cols         = 0;       // >= 2
    int       rows         = 0;       // >= 2 (cols*rows >= 6)
    double    hole_dia_mm  = 0.0;
    double    origin_x_mm  = 0.0;     // the (0,0) corner hole
    double    origin_y_mm  = 0.0;
    double    u_dx = 1.0, u_dy = 0.0; // column-axis direction (need not be unit)
    double    v_dx = 0.0, v_dy = 1.0; // row-axis direction
    double    pitch_u_mm   = 0.0;     // column spacing
    double    pitch_v_mm   = 0.0;     // row spacing
    gp_Dir    axis_dir     { 0.0, 0.0, -1.0 };
    double    depth_mm     = 0.0;
    bool      through_hole = true;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);   // empty — grammar in koo_re

}  // namespace rectangular_hole_grid
}  // namespace koocadcam::skill
