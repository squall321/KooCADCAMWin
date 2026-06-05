#pragma once
// @lat: [[engine/skills#horn_throat_conical_flare]]
//
// horn_throat_conical_flare — Acoustic horn throat / flare (slice 16, pro
// audio hardware).
//
// A compression-driver horn expands from a small throat (driver exit) to a
// large mouth (radiating aperture).  This skill machines a single conical
// flare into a flat-topped block: the throat opening is at the TOP face and
// the bore widens as it descends toward the mouth, cut with one
// pr::coneFrustum tool (rBottom = mouth radius, rTop = throat radius).
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. conical flare bore (single coneFrustum cut DOWN from top face)
//
// subfeature_count = 1.
//
// DFM:
//   DFM-INPUT  : every dimension must be > 0.
//   DFM-FLARE  : mouth_dia must be > throat_dia (a horn must expand).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace horn_throat_conical_flare {

constexpr const char* kSkillId = "horn_throat_conical_flare";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy        { 0.0, 0.0, 0.0 };
    double    throat_dia_mm    { 0.0 };   // small opening (top face)
    double    mouth_dia_mm     { 0.0 };   // large opening (bottom of flare)
    double    flare_length_mm  { 0.0 };   // axial depth of the flare
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace horn_throat_conical_flare
}  // namespace koocadcam::skill
