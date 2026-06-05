#pragma once
// @lat: [[engine/skills#htd_timing_pulley_teeth]]
//
// htd_timing_pulley_teeth — HTD / GT timing-belt pulley tooth-gap cut
// (3 mm / 5 mm / 8 mm curvilinear pitch) — power transmission.
//
// An HTD timing pulley is turned to a blank, then N rounded tooth gaps are
// cut around the outside diameter.  The HTD tooth has a curvilinear
// (rounded) root; each gap is modeled as a radial rounded pocket on the OD,
// spaced one curvilinear pitch apart on the pitch circle
//   PCD = (htd_pitch * tooth_count) / pi.
//
// Sub-features (SEQUENTIAL pr::cut via SetRotation, no compound boolean):
//   1..N. N tooth gaps removed from the blank OD, one per tooth.
//
// subfeature_count = tooth_count.
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-PT-PITCH  : htd_pitch_mm in { 3, 5, 8 } (standard HTD pitches).
//   DFM-PT-TEETH  : tooth_count >= 10.
//   DFM-PT-BLANK  : blank_outer_dia_mm must exceed the pitch circle.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace htd_timing_pulley_teeth {

constexpr const char* kSkillId = "htd_timing_pulley_teeth";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy          { 0.0, 0.0, 0.0 };
    double    htd_pitch_mm       = 0.0;   // 3 | 5 | 8
    int       tooth_count        = 0;     // N (>= 10)
    double    belt_width_mm      = 0.0;   // axial pulley face width
    double    tooth_depth_mm     = 0.0;   // radial tooth gap depth
    double    blank_outer_dia_mm = 0.0;   // turned-blank outside diameter
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace htd_timing_pulley_teeth
}  // namespace koocadcam::skill
