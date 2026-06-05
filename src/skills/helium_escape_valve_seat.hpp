#pragma once
// @lat: [[engine/skills#helium_escape_valve_seat]]
//
// helium_escape_valve_seat — Helium escape valve (HEV) seat for a dive watch
// case (slice 16, watch advanced complications).
//
// Saturation-diving watches use an automatic helium escape valve to vent
// helium that permeated into the case during decompression.  The case-side
// machining is:
//   - a central HEV bore (the vent passage through the case wall)
//   - a spring-seat counterbore (wider pocket seating the valve return spring)
//   - an AS568 O-ring groove (the static seal around the valve body)
//
// Sub-features (SEQUENTIAL pr::cut, no compound boolean):
//   1. HEV bore (narrow cylinder cut DOWN from top face)
//   2. spring-seat counterbore (wider cylinder)
//   3. AS568 O-ring groove (annular ring at the seat)
//
// subfeature_count = 3.
//
// DFM:
//   DFM-INPUT     : all dimensions must be > 0.
//   DFM-AS568     : o_ring_size_key must exist in central AS568 table.
//   DFM-SEAT      : spring_seat_dia_mm must exceed valve_bore_dia_mm.
//   DFM-DEPTH     : seat_depth_mm must be > 0 and ≤ 8 mm.

#include "Datum.hpp"
#include "Skill.hpp"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace helium_escape_valve_seat {

constexpr const char* kSkillId = "helium_escape_valve_seat";

struct Input
{
    FaceDatum   face_id;
    gp_Pnt      center_xy          { 0.0, 0.0, 0.0 };
    double      valve_bore_dia_mm   = 2.0;
    double      spring_seat_dia_mm  = 5.0;
    double      seat_depth_mm       = 3.0;
    std::string o_ring_size_key;                    // AS568 dash, e.g. "-011"
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace helium_escape_valve_seat
}  // namespace koocadcam::skill
