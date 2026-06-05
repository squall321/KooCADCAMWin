#pragma once
// @lat: [[engine/skills#thru_hull_seacock_flange]]
//
// thru_hull_seacock_flange — Thru-hull / seacock flange seat (slice 16, marine
// / boat hardware).
//
// A bronze seacock bolts to a thru-hull fitting through the hull.  The flange
// seat machined into the hull boss / backing block carries:
//   1. thru-hull bore   — central through-bore for the seacock spud / thread.
//   2. flange seat counterbore — wider flat seat the flange face lands on.
//   3. AS568 O-ring face groove — seals the flange to the seat.
//   4..6. three bolt holes on the bolt-circle diameter (PCD), placed at 120°
//         via gp_Trsf::SetRotation — SEQUENTIAL cuts, no compound boolean.
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. thru-hull bore
//   2. flange seat counterbore
//   3. AS568 O-ring face groove
//   4. bolt hole #1
//   5. bolt hole #2
//   6. bolt hole #3
//
// subfeature_count = 6.
//
// DFM:
//   DFM-INPUT     : every dimension must be > 0.
//   DFM-AS568     : o_ring_size_key must exist in central AS568 table.
//   DFM-MARINE-PCD: bolt_circle_dia must be > flange-seat clearance but
//                   < flange_dia (bolts must land on the flange, clearing bore).

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace thru_hull_seacock_flange {

constexpr const char* kSkillId = "thru_hull_seacock_flange";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy           { 0.0, 0.0, 0.0 };
    double      hull_bore_dia_mm    = 0.0;   // central thru-hull bore
    double      flange_dia_mm       = 0.0;   // flange seat counterbore dia
    double      bolt_circle_dia_mm  = 0.0;   // PCD of the 3 bolt holes
    std::string o_ring_size_key;             // from _as568_table.hpp ("-116"…)
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace thru_hull_seacock_flange
}  // namespace koocadcam::skill
