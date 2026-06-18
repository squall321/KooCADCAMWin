#pragma once
// @lat: [[engine/skills#coaxial_step_bore]]
//
// coaxial_step_bore — GENERATIVE side of the B1.2 grammar's step-bore pattern.
//
// re::recognizeCompounds recognises >=3 concentric full bores of distinct
// diameter and records, per step, {diameter_mm, depth_mm (from the part top),
// through}.  This skill regenerates the stack by boring each step's diameter
// from the entry face down to its cumulative depth (the deepest step may be
// through), so a recognised step bore becomes a replayable, EDITABLE step.
// Mirrors the other generative compounds (drills are composed from
// drill_hole::apply).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace coaxial_step_bore {

constexpr const char* kSkillId = "coaxial_step_bore";

struct Step
{
    double diameter_mm = 0.0;
    double depth_mm    = 0.0;     // from the entry face; ignored when through
    bool   through     = false;
};

struct Input
{
    FaceDatum entry_face = FaceByNormal{ gp_Dir(0.0, 0.0, 1.0) };
    double    center_x_mm = 0.0;
    double    center_y_mm = 0.0;
    gp_Dir    axis_dir    { 0.0, 0.0, -1.0 };
    std::vector<Step> steps;      // >= 3, distinct diameters
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);   // empty — grammar in koo_re

}  // namespace coaxial_step_bore
}  // namespace koocadcam::skill
