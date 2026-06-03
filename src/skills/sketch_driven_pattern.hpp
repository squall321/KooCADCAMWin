#pragma once
// @lat: [[engine/skills#sketch_driven_pattern]]
//
// sketch_driven_pattern — replicate a hole at N user-specified (x,y)
// positions on a host face (SolidWorks "Sketch Driven Pattern").
//
// apply():  for each (x,y) in positions, build a pr::cylinder normal to the
//           face plane and bundle them all into a single pr::cutMany.
//
// DFM gates:
//   DFM-INPUT  : positions.size() ≥ 1, hole_dia_mm > 0, hole_depth_mm > 0
//   DFM-002    : hole_dia_mm ≥ 0.8 mm
//   DFM-PATT-5 : every pair of (x,y) must be separated by > hole_dia_mm

#include "Datum.hpp"
#include "Skill.hpp"

#include <utility>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace sketch_driven_pattern {

constexpr const char* kSkillId = "sketch_driven_pattern";

struct Input
{
    double      hole_dia_mm   = 0.0;
    double      hole_depth_mm = 0.0;
    std::vector<std::pair<double,double>> positions;   // (x,y) deltas from face center
    FaceDatum   entry_face;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace sketch_driven_pattern
}  // namespace koocadcam::skill
