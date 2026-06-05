#pragma once
// @lat: [[engine/skills#harmonic_drive_flange_bore]]
//
// harmonic_drive_flange_bore — Strain-wave (harmonic) gear OUTPUT flange
// (slice 16, robotics / automation).
//
// The circular-spline output flange of a strain-wave gear set carries the
// robot joint's rotary output.  It presents a precision center bore (H7
// locating fit for the downstream link hub), a bolt circle that fastens the
// link to the flange, and two locating dowel holes that orient the link
// before the bolts are torqued.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. center bore (H7 via _iso286_fits.hpp)        — biggest feature first
//   2..N+1. N-bolt circle (gp_Trsf SetRotation)
//   N+2..N+3. 2 locating dowel holes (on dowel circle)
//
// subfeature_count = 1 + bolt_count + 2.
//
// DFM:
//   DFM-INPUT          : every dimension must be > 0.
//   DFM-ROBOTICS-COUNT : bolt_count in [3, 12].
//   DFM-ROBOTICS-PCD   : bolt_circle_dia_mm must exceed flange_bore_dia_mm
//                        (bolts must clear the center bore).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace harmonic_drive_flange_bore {

constexpr const char* kSkillId = "harmonic_drive_flange_bore";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy           { 0.0, 0.0, 0.0 };
    double    flange_bore_dia_mm  = 40.0;   // nominal H7 center bore
    double    bolt_circle_dia_mm  = 70.0;
    int       bolt_count          = 6;
    double    bolt_dia_mm         = 5.5;
    double    dowel_dia_mm        = 4.0;
    double    dowel_circle_dia_mm = 56.0;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace harmonic_drive_flange_bore
}  // namespace koocadcam::skill
