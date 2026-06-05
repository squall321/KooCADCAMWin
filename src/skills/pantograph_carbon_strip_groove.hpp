#pragma once
// @lat: [[engine/skills#pantograph_carbon_strip_groove]]
//
// pantograph_carbon_strip_groove — Pantograph collector carbon-strip mounting
// groove (slice 16, railway / rolling stock).
//
// The pantograph collector head carries replaceable carbon contact strips,
// seated in a long machined groove along the collector bar.  Retention clips
// snap into transverse slots cut along the groove floor.  The bar face has:
//   1. carbon-strip groove — a long rectangular pocket running along the bar.
//   2. N clip retention slots — narrow transverse slots spaced along the
//      groove that capture the strip retention clips.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. carbon-strip groove (long box cut DOWN from top face)
//   2..(1+clip_count)  N clip retention slots (boxes spaced along the groove)
//
// subfeature_count = 1 + clip_count.
//
// DFM:
//   DFM-INPUT  : every dimension must be > 0.
//   DFM-COUNT  : clip_count in [1, 24].
//   DFM-WIDTH  : clip_slot_width_mm must be smaller than groove_width_mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pantograph_carbon_strip_groove {

constexpr const char* kSkillId = "pantograph_carbon_strip_groove";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    strip_origin        { 0.0, 0.0, 0.0 };  // groove start XY
    double    strip_length_mm     = 1000.0;           // groove length (along X)
    double    groove_width_mm     = 30.0;
    double    groove_depth_mm     = 12.0;
    int       clip_count          = 6;                // [1, 24]
    double    clip_slot_width_mm  = 6.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pantograph_carbon_strip_groove
}  // namespace koocadcam::skill
