#pragma once
// @lat: [[engine/skills#solar_rail_splice_channel]]
//
// solar_rail_splice_channel — Rail splice/joiner channel for PV mounting rail
// (slice 16, solar / PV mounting).
//
// Adjacent mounting rails are joined end-to-end by an internal splice bar that
// drops into a machined channel; self-tapping screws then lock the splice to
// each rail.  This skill machines the channel plus a row of screw pilot holes.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. splice channel (box cut DOWN from top face)
//   N. self-tapping screw pilot holes (cylinders spaced along channel length)
//
// subfeature_count = 1 + screw_count.
//
// DFM:
//   DFM-INPUT   : every dimension must be > 0.
//   DFM-SCREWS  : screw_count in [1, 12].
//   DFM-PILOT   : screw_pilot_dia_mm must be < channel_width_mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace solar_rail_splice_channel {

constexpr const char* kSkillId = "solar_rail_splice_channel";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy          { 0.0, 0.0, 0.0 };
    double    channel_width_mm   = 12.0;
    double    channel_depth_mm   = 5.0;
    double    channel_length_mm  = 60.0;
    int       screw_count        = 4;
    double    screw_pilot_dia_mm = 3.5;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace solar_rail_splice_channel
}  // namespace koocadcam::skill
