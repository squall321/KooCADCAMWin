#pragma once
// @lat: [[engine/skills#pitch_bearing_bolt_circle]]
//
// pitch_bearing_bolt_circle — Wind turbine pitch-bearing flange bolt circle
// (slice 16, wind domain).
//
// The large pitch-bearing flange is fastened to the blade root / hub by a
// dense bolt circle.  This skill drills `bolt_count` clearance holes on a
// pitch-circle diameter (PCD) plus a central spindle bore — the kind of
// pattern machined on the bearing's mating flange face.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. central spindle bore (cylinder cut DOWN from top face)
//   2..N+1: N bolt holes placed via gp_Trsf::SetRotation around the center,
//           each cut INDIVIDUALLY against the running workpiece.
//
// subfeature_count = bolt_count + 1.
//
// DFM:
//   DFM-INPUT      : every dimension must be > 0.
//   DFM-BOLT-COUNT : bolt_count in [12, 120] (utility pitch bearing band).
//   DFM-PCD        : bolt-circle dia must clear the center bore.
//   DFM-STOCK      : bolt holes must sit inside the flange (PCD + bolt < OD).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace pitch_bearing_bolt_circle {

constexpr const char* kSkillId = "pitch_bearing_bolt_circle";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy            { 0.0, 0.0, 0.0 };
    double      bolt_circle_dia_mm   = 700.0;
    int         bolt_count           = 54;
    double      bolt_dia_mm          = 27.0;
    double      center_bore_dia_mm   = 400.0;
    double      bore_depth_mm        = 40.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace pitch_bearing_bolt_circle
}  // namespace koocadcam::skill
