#pragma once
// @lat: [[engine/skills#chainsaw_bar_chain_groove]]
//
// chainsaw_bar_chain_groove — chainsaw guide-bar chain-rail groove plus the
// sprocket-end arc cutout (slice 16, agricultural/forestry).
//
// The chainsaw guide bar is a thin flat plate; the cutting chain runs in a
// narrow central groove that's machined the full length of the bar.  One
// end (the "tail" / motor end) is closed; the other end (the "nose") is an
// arc relief that houses the chain-tip sprocket.  This compound skill cuts:
//
//   1. central groove (rectangular cross-section, closed at the tail end,
//      open at the nose), running the full bar_length
//   2. sprocket-end arc cutout — a semicircular relief at the nose so the
//      chain can wrap the tip sprocket
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. groove box (length = bar_length - sprocket_radius, narrow)
//   2. sprocket-end arc cylinder (radius = sprocket_radius)
//
// subfeature_count = 2.
//
// DFM:
//   DFM-INPUT  : every dimension must be > 0.
//   DFM-GROOVE : groove_width typical .050"/.058"/.063" range (1.0-2.0 mm).
//   DFM-DEPTH  : groove_depth typical 6-9 mm.
//   DFM-BAR    : sprocket_radius must be < bar_length / 4 (otherwise the
//                bar is shorter than the relief).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace chainsaw_bar_chain_groove {

constexpr const char* kSkillId = "chainsaw_bar_chain_groove";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    bar_axis_origin   { 0.0, 0.0, 0.0 };  // tail-end origin of bar
    double    bar_length_mm     = 400.0;            // typ 350-500 mm
    double    groove_width_mm   = 1.5;              // typ 1.3/1.5/1.6 mm
    double    groove_depth_mm   = 7.5;              // typ 7-8 mm
    double    sprocket_radius_mm = 18.0;            // chain-tip sprocket relief radius
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace chainsaw_bar_chain_groove
}  // namespace koocadcam::skill
