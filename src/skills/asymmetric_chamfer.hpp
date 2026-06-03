#pragma once
// @lat: [[engine/skills#asymmetric_chamfer]]
//
// asymmetric_chamfer — chamfer with different distances on the two faces
// sharing each selected edge.  OCCT's BRepFilletAPI_MakeChamfer two-distance
// Add() overload:
//     Add(double dis1, double dis2, const TopoDS_Edge& E, const TopoDS_Face& F)
// is used, where F identifies the side on which dis1 is measured.  For each
// selected edge we look up an adjacent face via TopExp::MapShapesAndAncestors
// and feed it as F.

#include "Datum.hpp"
#include "Skill.hpp"

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace asymmetric_chamfer {

constexpr const char* kSkillId = "asymmetric_chamfer";

struct Input
{
    double edge_z_band_min = 0.0;
    double edge_z_band_max = 0.0;
    double distance_a_mm   = 1.0;   // along face_a
    double distance_b_mm   = 1.0;   // along face_b
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace asymmetric_chamfer
}  // namespace koocadcam::skill
