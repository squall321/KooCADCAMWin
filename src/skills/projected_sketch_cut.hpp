#pragma once
// @lat: [[engine/skills#projected_sketch_cut]]
//
// projected_sketch_cut — project a 2D sketch polyline onto a target face's
// bounding-box plane (perpendicular to projection_dir), then extrude
// perpendicularly through the projection by cut_depth_mm and subtract from
// the workpiece.  Simpler-but-robust alternative to BRepProj_Projection
// which requires curve-on-face mapping that is finicky for non-planar
// targets.
//
// Inputs:
//   - target_face_id    : index of the face the sketch is targeting
//                         (used for DFM + recognition; geometry derived
//                          from projection_dir + cut_depth_mm)
//   - polygon           : XY polyline (pair<double,double>)
//   - projection_dir_xyz: world direction (gp_Dir) along which to project
//                         and cut
//   - cut_depth_mm      : how far to cut along projection_dir
//
// apply():  build closed wire from polygon at the workpiece bbox plane
// nearest to projection_dir origin → BRepBuilderAPI_MakeFace → MakePrism
// along projection_dir by cut_depth_mm → prim::cut.
//
// DFM:
//   - polygon ≥ 3 vertices
//   - cut_depth_mm > 0
//   - projection_dir not zero-length

#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace projected_sketch_cut {

constexpr const char* kSkillId = "projected_sketch_cut";

struct Input
{
    int                                   target_face_id    = -1;
    std::vector<std::pair<double,double>> polygon;
    gp_Dir                                projection_dir_xyz { 0.0, 0.0, -1.0 };
    double                                cut_depth_mm      = 5.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace projected_sketch_cut
}  // namespace koocadcam::skill
