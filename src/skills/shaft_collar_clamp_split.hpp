#pragma once
// @lat: [[engine/skills#shaft_collar_clamp_split]]
//
// shaft_collar_clamp_split — Clamp-style (split) shaft collar
// (bore + axial split slot + transverse clamp-screw bore) — power
// transmission.
//
// A clamp shaft collar grips a shaft by a split that is closed with a
// transverse cap screw.  It is machined from a turned ring blank with:
//   1. a central H7 bore (ISO 286 hole-basis, _iso286_fits.hpp)
//   2. an axial split slot through one wall (thin box, full axial length)
//   3. a transverse clamp-screw bore (M-thread pilot, _iso_thread_table.hpp)
//      crossing the split so the screw pulls the two halves together.
//
// Sub-features (SEQUENTIAL pr::cut, no overlapping-cutter compound boolean):
//   1. central H7 bore (big feature first)
//   2. axial split slot (thin radial box through the wall)
//   3. transverse clamp-screw pilot bore (perpendicular to the collar axis)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-PT-WALL   : collar_od_mm must exceed bore_dia_mm with usable wall.
//   DFM-PT-THREAD : clamp_thread_key must exist in the metric thread table.
//   DFM-PT-SPLIT  : split_width_mm must be < wall thickness.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace shaft_collar_clamp_split {

constexpr const char* kSkillId = "shaft_collar_clamp_split";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      axis_origin   { 0.0, 0.0, 0.0 };  // collar bore axis base point
    double      bore_dia_mm   = 0.0;              // shaft bore (H7 applied)
    double      collar_od_mm  = 0.0;              // collar outside diameter
    double      split_width_mm = 0.0;             // axial split slot width
    std::string clamp_thread_key;                 // "M5" | "M6" | ... metric
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace shaft_collar_clamp_split
}  // namespace koocadcam::skill
