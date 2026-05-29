#pragma once
// @lat: [[engine/skills#cam_actuated_slider]]
//
// cam_actuated_slider — linear-slider channel coupled to a cam pivot.
//
// A cam-driven slider has:
//   - a LINEAR SLOT (the slider's travel path)
//   - a CAM PIVOT BORE (where the rotating cam is mounted)
//   - a CAM NOTCH (a key slot on the slider that engages the cam lobe)
//
// COMPOUND CHAIN (≥ 3 primitive cuts/fuses):
//   1. LINEAR SLOT cut (slider channel along x_axis)
//   2. PIVOT BORE cut (cylindrical, perpendicular to entry)
//   3. CAM NOTCH cut (small rectangular pocket transverse to slot)
//
// Context-aware: queries wp.faceCount() and wp.bbox() to:
//   - place the pivot bore at (bbox center_x ± offset) when not specified
//   - select pivot_offset_mm based on the longest bbox dimension
//
// Spec table (cam_class → standard cam-slider geometry):
//   "MICRO_M3"    : slot_w 3, pivot_dia 3, notch_w 1.5, throw 4
//   "STD_M5"      : slot_w 5, pivot_dia 5, notch_w 2.5, throw 8
//   "HEAVY_M8"    : slot_w 8, pivot_dia 8, notch_w 4.0, throw 12
//
// DFM gates:
//   DFM-002       : slot/notch widths ≥ 0.8 mm
//   DFM-CAM-CLR   : pivot bore must not overlap slot walls
//   DFM-SPEC      : cam_class must be in spec table

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace cam_actuated_slider {

constexpr const char* kSkillId = "cam_actuated_slider";

struct Input
{
    FaceDatum    entry_face;
    double       slot_start_x_mm = 0.0;
    double       slot_start_y_mm = 0.0;
    double       slot_end_x_mm   = 0.0;
    double       slot_end_y_mm   = 0.0;
    gp_Dir       axis_dir        { 0.0, 0.0, -1.0 };
    double       slot_depth_mm   = 0.0;
    double       pivot_offset_mm = 0.0;   // pivot is offset perp from slot
    std::string  cam_class       = "STD_M5";
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

struct CamSpec {
    const char* key;
    double      slot_w_mm;
    double      pivot_dia_mm;
    double      notch_w_mm;
    double      throw_mm;
};
const CamSpec* lookupCamSpec(const std::string& key);

}  // namespace cam_actuated_slider
}  // namespace koocadcam::skill
