#pragma once
// @lat: [[engine/skills#flange_coupling_bolt_circle]]
//
// flange_coupling_bolt_circle — Rigid flange coupling machining
// (center bore + DIN 6885 keyway + bolt circle) — power transmission.
//
// A rigid flange coupling hub is machined from a turned blank with:
//   1. a central H7 shaft bore (ISO 286 hole-basis fit, _iso286_fits.hpp)
//   2. a DIN 6885 parallel keyway sized for the bore (_keyway_table.hpp)
//   3. N bolt holes on a bolt circle (SetRotation, sequential cuts)
//
// Sub-features (SEQUENTIAL pr::cut, no overlapping-cutter compound boolean):
//   1.        central H7 bore (cylinder through)
//   2.        DIN 6885 keyway slot (box from the bore outward into the hub)
//   3..N+2.   N bolt holes on the bolt circle (SetRotation)
//
// subfeature_count = 1 + 1 + bolt_count.
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-PT-KEY    : bore_dia_mm must be within DIN 6885 table range.
//   DFM-PT-PCD    : bolt_circle_dia_mm must exceed bore_dia_mm so bolts
//                   clear the bore.
//   DFM-PT-BOLTS  : bolt_count >= 3.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace flange_coupling_bolt_circle {

constexpr const char* kSkillId = "flange_coupling_bolt_circle";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy          { 0.0, 0.0, 0.0 };
    double    bore_dia_mm        = 0.0;   // nominal shaft bore (H7 applied)
    double    key_length_mm      = 0.0;   // axial keyway length
    double    bolt_circle_dia_mm = 0.0;   // bolt circle diameter (PCD)
    int       bolt_count         = 0;     // number of coupling bolts
    double    bolt_dia_mm        = 0.0;   // bolt clearance hole diameter
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace flange_coupling_bolt_circle
}  // namespace koocadcam::skill
