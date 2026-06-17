#pragma once
// @lat: [[engine/skills#bolt_circle_pattern]]
//
// bolt_circle_pattern — GENERATIVE side of the B1.2 grammar compound.
//
// The reverse-engineering grammar (re::recognizeCompounds) already RECOGNISES a
// ring of equal, parallel-axis holes as a "bolt_circle_pattern" with parametric
// recovered_params {hole_count, bolt_circle_dia_mm, hole_dia_mm, center_x/y,
// axis_dir}.  This skill is the missing other half: it GENERATES that ring, so a
// recognised bolt circle becomes a replayable, EDITABLE process step — change
// hole_count or bolt_circle_dia_mm and re-execute to regenerate the pattern.
//
// Synthesis: drills `hole_count` equal holes evenly spaced on a circle of
// diameter bolt_circle_dia_mm centred at (center_x_mm, center_y_mm), composing
// drill_hole::apply per hole (so entry/through resolution is shared and proven).
//
// Recognition lives in the grammar layer (koo_re), not here — recognize()
// returns empty so the skill registry does not double-count.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace bolt_circle_pattern {

constexpr const char* kSkillId = "bolt_circle_pattern";

struct Input
{
    FaceDatum entry_face = FaceByNormal{ gp_Dir(0.0, 0.0, 1.0) };
    int       hole_count        = 0;       // >= 3
    double    bolt_circle_dia_mm = 0.0;    // pitch-circle diameter
    double    hole_dia_mm       = 0.0;
    double    center_x_mm       = 0.0;
    double    center_y_mm       = 0.0;
    gp_Dir    axis_dir          { 0.0, 0.0, -1.0 };  // hole drilling direction
    double    depth_mm          = 0.0;     // ignored when through_hole
    bool      through_hole      = true;    // bolt circles are typically through
    double    start_angle_deg   = 0.0;     // angle of the first hole (0 = +X)
};

// Synthesis: drill the ring; returns the new workpiece + the compound signature.
SkillOutput apply(const Workpiece& wp, const Input& in);

// DFM validation (hole count, diameters, pitch sanity).
DFMReport validate(const Workpiece& wp, const Input& in);

// Recognition is performed by re::recognizeCompounds (koo_re); empty here.
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace bolt_circle_pattern
}  // namespace koocadcam::skill
