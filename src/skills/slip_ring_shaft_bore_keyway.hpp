#pragma once
// @lat: [[engine/skills#slip_ring_shaft_bore_keyway]]
//
// slip_ring_shaft_bore_keyway — Wind turbine slip-ring shaft bore + keyway
// (slice 16, wind domain).
//
// The slip ring transmits pitch-system power / signals from the rotating hub
// to the stationary nacelle.  Its rotor hub is keyed to the main shaft and
// routes cable through an axial channel.  This skill machines the central
// shaft bore, a DIN 6885 parallel keyway sized to the bore diameter, and a
// cable routing channel (box pocket) along the top face.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. central shaft bore (cylinder cut DOWN from top face)
//   2. DIN 6885 keyway (box pocket on the bore wall, key width from table)
//   3. cable routing channel (box pocket along the top face)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT    : every dimension must be > 0.
//   DFM-KEYWAY   : shaft_bore_dia must map to a DIN 6885 parallel band.
//   DFM-KEY-POS  : key_position_z + key_length must fit within the bore depth.
//   DFM-CHANNEL  : cable channel width/depth > 0 and channel narrower than bore.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace slip_ring_shaft_bore_keyway {

constexpr const char* kSkillId = "slip_ring_shaft_bore_keyway";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      axis_origin            { 0.0, 0.0, 0.0 };  // XY of bore axis
    double      shaft_bore_dia_mm      = 60.0;
    double      key_position_z_mm      = 5.0;   // below top face to keyway start
    double      key_length_mm          = 40.0;
    double      cable_channel_width_mm = 12.0;
    double      cable_channel_depth_mm = 10.0;
};

SkillOutput apply   (const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace slip_ring_shaft_bore_keyway
}  // namespace koocadcam::skill
