#pragma once
// @lat: [[engine/skills#guitar_truss_rod_channel]]
//
// guitar_truss_rod_channel — straight truss rod channel under fretboard.
//
// Slice 16 — audio / musical instrument hardware.
//
// Sub-features (sequential CUTs):
//   1. straight channel slot (rectangular box) along rod axis, full length
//   2. nut hex recess at the "nut" end (where the adjustment wrench engages)
//   3. anchor recess at the far end (captures the rod end-anchor block)
//
// subfeature_count = 3 (channel + nut recess + anchor recess).
//
// All cuts are sequential pr::cut on the previous result.
//
// DFM gates:
//   DFM-INPUT    : all positive dims
//   DFM-CHANNEL  : channel width in [4, 9] mm, depth in [4, 12] mm
//   DFM-LENGTH   : rod_length in [300, 700] mm (typ guitar 400-500)
//   DFM-NUT      : nut recess dia must be < channel width + 2 mm to fit
//   DFM-ANCHOR   : anchor recess width <= channel width + 3 mm

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace guitar_truss_rod_channel {

constexpr const char* kSkillId = "guitar_truss_rod_channel";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      rod_axis_origin       { 0.0, 0.0, 0.0 };  // start of rod (nut end)
    gp_Dir      rod_axis_dir          { 1.0, 0.0, 0.0 };  // direction toward body end
    double      rod_length_mm         = 450.0;   // typ 400-500 mm
    double      rod_channel_width_mm  = 6.0;     // typ 5-7 mm
    double      rod_channel_depth_mm  = 7.0;     // typ 6-8 mm
    double      nut_recess_dia_mm     = 6.0;     // typ 5-8 mm (hex wrench socket)
    double      nut_recess_depth_mm   = 10.0;
    double      end_anchor_recess_w_mm = 8.0;    // anchor block recess width
    double      end_anchor_recess_l_mm = 12.0;   // anchor block recess length
    double      end_anchor_recess_d_mm = 8.0;    // anchor block recess depth
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace guitar_truss_rod_channel
}  // namespace koocadcam::skill
