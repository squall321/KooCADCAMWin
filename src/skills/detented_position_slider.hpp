#pragma once
// @lat: [[engine/skills#detented_position_slider]]
//
// detented_position_slider — linear slider channel with N indexing detents.
//
// Detented position sliders provide tactile click-stop positioning for
// switches, selectors and rotation locks.  Geometry:
//   - LINEAR SLOT (slider travel channel)
//   - N DETENT DIMPLES (small hemispherical indents along the slot floor)
//
// COMPOUND CHAIN (≥ 1 slot cut + N dimple cuts; for N ≥ 2 we cut ≥ 3 primitives):
//   1. linear SLOT cut
//   2..N+1.  for k = 0..N-1: cut a small cylindrical dimple at fractional
//            position k/(N-1) along the slot floor
//
// Spec table (detent_class → standard detent geometry):
//   "FINE_3"   :  3 positions, dimple_dia 1.5, dimple_depth 0.4
//   "STD_5"    :  5 positions, dimple_dia 2.0, dimple_depth 0.6
//   "MED_7"    :  7 positions, dimple_dia 2.5, dimple_depth 0.8
//   "COARSE_10": 10 positions, dimple_dia 3.0, dimple_depth 1.0
//
// Context-aware: queries wp.bbox() to constrain the detent count so that
// dimples don't overlap (detent pitch ≥ 2 × dimple_dia).
//
// DFM gates:
//   DFM-002       : slot width ≥ 0.8 mm
//   DFM-DIM-PITCH : detent pitch ≥ 2 × dimple_dia (else dimples overlap)
//   DFM-SPEC      : detent_class must be in spec table

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace detented_position_slider {

constexpr const char* kSkillId = "detented_position_slider";

struct Input
{
    FaceDatum    entry_face;
    double       slot_start_x_mm = 0.0;
    double       slot_start_y_mm = 0.0;
    double       slot_end_x_mm   = 0.0;
    double       slot_end_y_mm   = 0.0;
    gp_Dir       axis_dir        { 0.0, 0.0, -1.0 };
    double       slot_width_mm   = 0.0;
    double       slot_depth_mm   = 0.0;
    std::string  detent_class    = "STD_5";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

struct DetentSpec {
    const char* key;
    int         detent_count;
    double      dimple_dia_mm;
    double      dimple_depth_mm;
};
const DetentSpec* lookupDetentSpec(const std::string& key);

}  // namespace detented_position_slider
}  // namespace koocadcam::skill
