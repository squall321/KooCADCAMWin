#pragma once
// @lat: [[engine/skills#extrude_cut_from_sketch]]
//
// extrude_cut_from_sketch — SolidWorks-style sketch-driven cut extrusion.
//
// Dual of extrude_boss_from_sketch.  Same inputs (face, polygon, height,
// draft) plus a `through_all` flag — when true the cut depth is grown to the
// workpiece bounding box's diagonal so the cut punches all the way through
// regardless of opposing geometry.
//
// apply():  build face from polygon → prism along the INTO-face normal
// (i.e. -face_normal for the cut direction) by `height_mm` (or bbox diag if
// through_all) → prim::cut from workpiece.
//
// DFM mirrors the boss version, with the addition that when through_all is
// false the cut depth must stay within the local stock thickness.

#include "Datum.hpp"
#include "Skill.hpp"

#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace extrude_cut_from_sketch {

constexpr const char* kSkillId = "extrude_cut_from_sketch";

struct Input
{
    FaceDatum                            entry_face;
    std::vector<std::pair<double,double>> polygon;
    double                               height_mm        = 5.0;
    double                               draft_angle_deg  = 0.0;
    bool                                 through_all      = false;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace extrude_cut_from_sketch
}  // namespace koocadcam::skill
