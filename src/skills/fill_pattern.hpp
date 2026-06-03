#pragma once
// @lat: [[engine/skills#fill_pattern]]
//
// fill_pattern — fill a face region with a small repeated cut.
//
// Two fill types:
//   "grid" : axis-aligned grid of pitch_mm spacing
//   "hex"  : honeycomb lattice with pitch_mm row spacing
//
// apply():  walk the face's XY bbox, compute centers, drill each via
//           pr::cylinder + pr::cutMany.  Margin shrinks the active fill area.
//
// DFM gates:
//   DFM-INPUT  : source_hole_dia_mm > 0, source_depth_mm > 0,
//                pitch_mm > source_hole_dia_mm, margin_mm ≥ 0
//   DFM-002    : source_hole_dia_mm ≥ 0.8 mm
//   DFM-PATT-4 : at least 1 instance must fit inside the active region
//   DFM-INPUT  : fill_type must be "hex" or "grid"

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>

namespace koocadcam::skill {

class Workpiece;

namespace fill_pattern {

constexpr const char* kSkillId = "fill_pattern";

struct Input
{
    FaceDatum   face;
    double      source_hole_dia_mm = 0.0;
    double      source_depth_mm    = 0.0;
    std::string fill_type          = "grid";  // "grid" | "hex"
    double      pitch_mm           = 0.0;
    double      margin_mm          = 0.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace fill_pattern
}  // namespace koocadcam::skill
