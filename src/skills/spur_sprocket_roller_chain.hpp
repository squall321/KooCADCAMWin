#pragma once
// @lat: [[engine/skills#spur_sprocket_roller_chain]]
//
// spur_sprocket_roller_chain — Roller-chain sprocket tooth-gap cut
// per ANSI/ASME B29.1 (chains #25 / #35 / #40) — power transmission.
//
// A roller-chain sprocket is machined from a turned blank disc by cutting
// N roller-seating gaps around the rim.  Each gap is the pocket where one
// chain roller seats; the gaps are spaced one chain pitch apart on the
// pitch circle, whose diameter is  PCD = pitch / sin(pi / N).
//
// Sub-features (SEQUENTIAL pr::cut via SetRotation, no compound boolean):
//   1..N. N roller-seating gaps removed from the blank rim, one per tooth
//         (each = a cylinder pocket centered on the pitch circle).
//
// subfeature_count = tooth_count.
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-PT-TEETH  : tooth_count >= 9 (ANSI minimum practical sprocket).
//   DFM-PT-BLANK  : blank_outer_dia_mm must exceed the pitch circle so a
//                   tooth tip remains between adjacent roller seats.
//   DFM-PT-ROLLER : roller_dia_mm must be < chain_pitch_mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spur_sprocket_roller_chain {

constexpr const char* kSkillId = "spur_sprocket_roller_chain";

struct Input
{
    FaceDatum face_id;
    gp_Pnt    center_xy          { 0.0, 0.0, 0.0 };
    double    chain_pitch_mm     = 0.0;   // ANSI #25=6.35, #35=9.525, #40=12.70
    double    roller_dia_mm      = 0.0;   // ANSI #25=3.30, #35=5.08, #40=7.92
    int       tooth_count        = 0;     // N (>= 9)
    double    blank_outer_dia_mm = 0.0;   // turned-blank outside diameter
    double    face_width_mm      = 0.0;   // axial sprocket thickness
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace spur_sprocket_roller_chain
}  // namespace koocadcam::skill
