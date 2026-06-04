#pragma once
// @lat: [[engine/skills#radiator_finger_array]]
//
// radiator_finger_array — tube + finger-fin array (HVAC / fluid).
//
// Slice 15 — HVAC/fluid.
//
// Sub-features (sequential cuts, no compound-boolean — slice-14 root cause):
//   1. inlet tube bore  (cylinder cut along +X axis)
//   2. outlet tube bore (cylinder cut along +X axis, offset on +Y)
//   3..(2+N). N thin parallel fin cuts (box pattern, between the two tubes,
//             cuts the air gaps that separate adjacent fins).
//
// subfeature_count = 2 + (fin_count - 1)   (N-1 air gaps between N fins).
//
// DFM:
//   DFM-FIN-THK : fin_thickness < 0.15 mm (cutter / stamp floor).
//   DFM-INPUT   : geometric inputs must be > 0.
//   DFM-FIN-N   : fin_count must be >= 2.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Dir.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace radiator_finger_array {

constexpr const char* kSkillId = "radiator_finger_array";

struct Input
{
    FaceDatum   face_id;
    int         fin_count          = 10;     // N
    double      fin_pitch_mm       = 3.0;    // centre-to-centre Y
    double      fin_thickness_mm   = 0.2;    // fin wall thickness (default 0.2)
    double      fin_length_mm      = 30.0;   // tube length along X
    double      fin_height_mm      = 20.0;   // fin height along Z
    double      tube_dia_mm        = 8.0;    // both inlet and outlet tube bores
};

SkillOutput  apply   (const Workpiece& wp, const Input& in);
DFMReport    validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace radiator_finger_array
}  // namespace koocadcam::skill
