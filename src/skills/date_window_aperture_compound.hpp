#pragma once
// @lat: [[engine/skills#date_window_aperture_compound]]
//
// date_window_aperture_compound — Date window aperture in a watch dial /
// dial plate (slice 16, watch advanced complications).
//
// The date complication shows the day number through a rectangular aperture
// cut in the dial.  The dial-plate machining is:
//   - a rectangular through-cut aperture (the window itself)
//   - a beveled frame around the aperture (a wider, shallow chamfer step so
//     the printed frame / applied surround sits proud)
//   - a glass / crystal step recess (a shallow rectangular rebate so the
//     date magnifier or covering glass drops in flush)
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. window aperture (rectangular through box cut DOWN from top face)
//   2. beveled frame (wider, shallow box step around the aperture)
//   3. glass step recess (widest, shallow box rebate at the surface)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT     : window_len/wid, glass_step_depth must be > 0.
//   DFM-BEVEL     : bevel_mm must be in (0, window_wid/2).
//   DFM-MARGIN    : glass_step_margin_mm must be > bevel_mm.
//   DFM-STEP      : glass_step_depth_mm must be < stock thickness.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace date_window_aperture_compound {

constexpr const char* kSkillId = "date_window_aperture_compound";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy            { 0.0, 0.0, 0.0 };
    double      window_len_mm         = 4.0;   // along X
    double      window_wid_mm         = 3.0;   // along Y
    double      bevel_mm              = 0.4;   // frame chamfer width
    double      glass_step_depth_mm   = 0.6;   // glass rebate depth
    double      glass_step_margin_mm  = 1.0;   // glass rebate margin beyond window
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace date_window_aperture_compound
}  // namespace koocadcam::skill
