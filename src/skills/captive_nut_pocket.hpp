#pragma once
// @lat: [[engine/skills#captive_nut_pocket]]
//
// captive_nut_pocket — COMPOUND skill for a captive-nut retention pocket
// (a "T-nut" or "press-in-nut" slot machined into a sheet/plate that lets
// you slide a hex nut in from one edge and trap it for blind threading).
//
// Chains:
//   sub-feature 1: hexagonal pocket (flat-to-flat = nut_AF + slip fit)
//   sub-feature 2: rectangular access slot extending from the hex pocket
//                  to one edge (entry channel for the nut to slide in)
//   sub-feature 3: small spring-detent relief at the back of the pocket
//                  (a tiny radial cylinder cut so the nut snaps into place)
//
// All cuts are at the same depth (= nut height + 0.2 mm) and combined into
// one Boolean cut.
//
// ISO 4032 metric hex-nut across-flats (AF) lookup:
//
//   nut spec | AF (mm) | height (mm)
//   ---------+---------+-------------
//   M2       | 4.0     | 1.6
//   M2.5     | 5.0     | 2.0
//   M3       | 5.5     | 2.4
//   M4       | 7.0     | 3.2
//   M5       | 8.0     | 4.0
//   M6       | 10.0    | 5.0
//   M8       | 13.0    | 6.5
//
// DFM:
//   DFM-NUT-SIZE   : nut_size unknown
//   DFM-NUT-DEPTH  : stock too thin
//   DFM-NUT-EDGE   : entry channel exits beyond stock extent
//   DFM-002        : derived slot width < 0.8 mm (impossible — guards table)
//
// Recognize: metadata replay, confidence 1.0.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace captive_nut_pocket {

constexpr const char* kSkillId = "captive_nut_pocket";

struct Input
{
    FaceDatum   entry_face;
    double      position_x_mm = 0.0;       // center of hex pocket
    double      position_y_mm = 0.0;
    gp_Dir      axis_dir       { 0.0, 0.0, -1.0 };
    std::string nut_size;                  // "M2".."M8"
    double      slip_fit_mm    = 0.2;      // AF + slip = pocket flat-to-flat
    std::string entry_edge     = "+x";     // "+x" | "-x" | "+y" | "-y"
};

double nutAcrossFlatsFor (const std::string& size);
double nutHeightFor      (const std::string& size);

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace captive_nut_pocket
}  // namespace koocadcam::skill
